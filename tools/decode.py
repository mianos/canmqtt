#!/usr/bin/env python3
"""Decode pasted mqttcan frames offline, using the same data/signals.json the
board runs. Mirrors SignalTable::decode so a log can be read without the bike.

  tools/decode.py < capture.log
  echo '{"id":"0x2BC","data":"FF534E000026F862"}' | tools/decode.py
Accepts the JSON the board publishes, or bare "2BC#FF534E..." candump text.
"""
import json, os, re, sys

HERE = os.path.dirname(os.path.abspath(__file__))
TBL = json.load(open(os.path.join(HERE, '..', 'data', 'signals.json')))
BASE = TBL.get('byte_base', 0)
NOISE = {int(k, 16): int(v, 16) for k, v in TBL.get('noise', {}).items()
         if not k.startswith('_')}


def nib(byte, kind):
    return byte >> 4 if kind == 'high' else byte & 0x0F if kind == 'low' else byte


def decode(cid, data):
    frame = TBL['frames'].get(f'{cid:X}')
    if not frame:
        return None, []
    out = []
    for s in frame['signals']:
        if 'parts' in s:
            key = ','.join(
                f'{nib(data[p["byte"] - BASE], p.get("nibble")):X}'
                if p.get('nibble') else f'{data[p["byte"] - BASE]:02X}'
                for p in s['parts'])
            out.append((s['name'], s['map'].get(key, f'unmapped_0x{key}'), ''))
            continue
        idx = [b - BASE for b in s['bytes']]
        if any(i >= len(data) for i in idx):
            continue
        raw = 0
        for n, i in enumerate(idx):
            raw |= data[i] << (8 * n)
        if len(idx) == 1 and s.get('nibble'):
            raw = nib(raw, s['nibble'])
        if 'invalid' in s and raw == s['invalid']:
            out.append((s['name'], '<no reading>', 'sentinel suppressed'))
            continue
        if 'map' in s:
            w = 1 if s.get('nibble') else len(idx) * 2
            key = f'{raw:0{w}X}'
            out.append((s['name'], s['map'].get(key, s.get('default') or f'unmapped_0x{key}'), ''))
        else:
            v = raw * s.get('scale', 1.0) + s.get('offset', 0.0)
            v = round(v, s.get('decimals', 0))
            out.append((s['name'], f'{v:g}', s.get('unit', '')))
    return frame.get('name', ''), out


for line in sys.stdin:
    m = re.search(r'"id"\s*:\s*"0x([0-9A-Fa-f]+)".*?"data"\s*:\s*"([0-9A-Fa-f]+)"', line) \
        or re.search(r'\b([0-9A-Fa-f]{3,8})#([0-9A-Fa-f]+)', line)
    if not m:
        continue
    cid, hexs = int(m.group(1), 16), m.group(2)
    data = bytes.fromhex(hexs)
    name, sigs = decode(cid, data)
    mask = NOISE.get(cid, 0)
    tag = f'  [muted bytes {mask:02X}]' if mask else ''
    print(f'0x{cid:X} {hexs}  ({name or "not in table"}){tag}')
    for n, v, u in sigs:
        print(f'      {n:<22} {v} {u}'.rstrip())
    if not sigs:
        print('      (no signals defined for this ID)')
