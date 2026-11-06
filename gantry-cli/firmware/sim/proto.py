# Emulate the Arduino IDE: insert prototypes for all top-level functions just above the first function definition.
import re, sys
src = open(sys.argv[1]).read().split('\n')
pat = re.compile(r'^((?!(?:if|for|while|switch|return|else|struct|enum)\b)[A-Za-z_][\w\s\*&:<>]*?\b[A-Za-z_]\w*\s*\([^()]*\))\s*\{')
first, protos, inraw = None, [], False
for i, l in enumerate(src):
    if 'R"HTML(' in l: inraw = True
    if inraw:
        if ')HTML"' in l: inraw = False
        continue
    m = pat.match(l)
    if m and '=' not in l.split('(')[0]:
        if first is None: first = i
        protos.append(m.group(1).strip() + ';')
out = src[:first] + ['// --- auto prototypes (Arduino IDE emulation) ---'] + protos + src[first:]
open(sys.argv[2], 'w').write('\n'.join(out))
print(f"{sys.argv[1].split('/')[-1]}: {len(protos)} prototypes above line {first+1}: " + ' '.join(p.split('(')[0].split()[-1] for p in protos))
