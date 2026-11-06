#!/usr/bin/env python3
"""Optional convenience CLI: discretize a continuous C(s) into b/a for control_params.yaml.

Not a hard dependency of the control loop -- if you derive your own
difference equation by hand, you don't need this or scipy at all. It's just
a shortcut for scipy.signal.cont2discrete.

Usage:
    python3 discretize_helper.py --num 1 5 --den 1 2 5 --ts 0.01 --method tustin

    --num / --den : continuous-domain C(s) numerator/denominator coefficients,
                    highest power of s first (same convention as scipy).
    --ts          : sample time [s], matching the control_params.yaml Ts you plan to use.
    --method      : 'tustin' (bilinear), 'zoh', or 'matched' (matched Z-transform).

Prints the resulting discrete b/a coefficients, ready to paste into
control_params.yaml as e.g. `control.x.b` / `control.x.a`.

Requires: pip install scipy numpy (optional -- see package.xml).
"""

import argparse


def discretize(num, den, ts, method):
    from scipy.signal import cont2discrete
    dnum, dden, _ = cont2discrete((num, den), ts, method=method)
    return dnum.flatten().tolist(), dden.flatten().tolist()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--num', type=float, nargs='+', required=True,
                         help='Continuous C(s) numerator coefficients, highest power first')
    parser.add_argument('--den', type=float, nargs='+', required=True,
                         help='Continuous C(s) denominator coefficients, highest power first')
    parser.add_argument('--ts', type=float, required=True, help='Sample time [s]')
    parser.add_argument('--method', choices=['tustin', 'zoh', 'matched'], default='tustin')
    args = parser.parse_args()

    b, a = discretize(args.num, args.den, args.ts, args.method)

    print(f'Discretized ({args.method}, Ts={args.ts}s):')
    print(f'  b: {b}')
    print(f'  a: {a}')


if __name__ == '__main__':
    main()
