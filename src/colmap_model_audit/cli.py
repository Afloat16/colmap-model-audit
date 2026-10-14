import argparse
import sys
from .common import emit, code
from .core import audit
from .model import read_model


def main(argv=None):
    parser=argparse.ArgumentParser(description='Read-only COLMAP cameras/images/points3D text-model audit.')
    parser.add_argument('model')
    parser.add_argument('--images-root')
    parser.add_argument('--compare-model')
    parser.add_argument('--min-overlap',type=int,default=3)
    parser.add_argument('--max-reprojection-px',type=float,default=4.0)
    parser.add_argument('--strict',action='store_true')
    parser.add_argument('--output')
    args=parser.parse_args(argv)
    try:
        report=audit(read_model(args.model),args.images_root,read_model(args.compare_model) if args.compare_model else None,
                     args.min_overlap,args.max_reprojection_px)
        emit(report,args.output)
        return code(report,args.strict)
    except (OSError,ValueError,TypeError,KeyError) as exc:
        print(f'Input/report error: {exc}',file=sys.stderr)
        return 2
