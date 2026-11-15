"""Compatibility entry point for the current complete RSI grasp workspace."""
import argparse
from tests.browser_rsi_ui import check

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--url',default='http://127.0.0.1:8891')
    parser.add_argument('--output',required=True)
    args=parser.parse_args();check(args.url,args.output)
