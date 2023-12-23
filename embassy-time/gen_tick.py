import os
from glob import glob

abspath = os.path.abspath(__file__)
dname = os.path.dirname(abspath)
os.chdir(dname)

ticks = []
for i in range(10):
    ticks.append(10**i)
for i in range(1, 25):
    ticks.append(2**i)
for i in range(1, 20):
    ticks.append(2**i * 1000)
for i in range(1, 20):
    ticks.append(2**i * 10000)
for i in range(1, 10):
    ticks.append(2**i * 1000000)
    ticks.append(2**i * 9 // 8 * 1000000)
    ticks.append(2**i * 3 // 2 * 1000000)
for i in range(1, 30):
    ticks.append(10 * i * 1_000_000)
for i in range(15, 50):
    ticks.append(20 * i * 1_000_000)

seen = set()
ticks = sorted([x for x in ticks if not (x in seen or seen.add(x))])

# ========= Update Cargo.toml

things = [(hz, f'tick-hz-{hz:_}') for hz in ticks]

SEPARATOR_START = '# BEGIN TICKS\n'
SEPARATOR_END = '# END TICKS\n'
HELP = '# Generated by gen_tick.py. DO NOT EDIT.\n'
with open('Cargo.toml', 'r') as f:
    data = f.read()
before, data = data.split(SEPARATOR_START, maxsplit=1)
_, after = data.split(SEPARATOR_END, maxsplit=1)

data = before + SEPARATOR_START + HELP
for freq, feature in things:
    if freq >= 1_000_000_000:
        freq_human = f"{freq / 1_000_000_000}GHz"
    elif freq >= 1_000_000:
        freq_human = f"{freq / 1_000_000}MHz"
    elif freq >= 1_000:
        freq_human = f"{freq / 1000}kHz"
    else:
        freq_human = f"{freq}Hz"

    data += f"## {freq_human} Tick Rate\n"
    data += f"{feature} = []\n"
data += SEPARATOR_END + after

with open('Cargo.toml', 'w') as f:
    f.write(data)

# ========= Update src/tick.rs

with open('src/tick.rs', 'w') as f:

    f.write('// Generated by gen_tick.py. DO NOT EDIT.\n\n')
    for hz in ticks:
        f.write(
            f'#[cfg(feature = "tick-hz-{hz:_}")] pub const TICK_HZ: u64 = {hz:_};\n')
    f.write('#[cfg(not(any(\n')
    for hz in ticks:
        f.write(f'feature = "tick-hz-{hz:_}",\n')
    f.write(')))] pub const TICK_HZ: u64 = 1_000_000;')


os.system('rustfmt src/tick.rs')
