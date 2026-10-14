"""Original reader for the documented legacy COLMAP text triples."""
from pathlib import Path
from .common import number

PARAMETERS={'SIMPLE_PINHOLE':3,'PINHOLE':4,'SIMPLE_RADIAL':4,'RADIAL':5,'OPENCV':8,
            'OPENCV_FISHEYE':8,'FULL_OPENCV':12,'FOV':5,'SIMPLE_RADIAL_FISHEYE':4,
            'RADIAL_FISHEYE':5,'THIN_PRISM_FISHEYE':12}


def lines(path):
    with Path(path).open('rb') as stream: raw=stream.read(64*1024*1024+1)
    if len(raw)>64*1024*1024: raise ValueError('Model text file exceeds 64 MiB.')
    return raw.decode('utf-8-sig').splitlines()


def nonnegative(value,name):
    parsed=int(value)
    if parsed<0: raise ValueError(f'{name} must be nonnegative.')
    return parsed


def floats(values):
    return [number(float(value),'model number') for value in values]


def insert(mapping,ident,value,kind):
    if ident in mapping: raise ValueError(f'Duplicate {kind} ID {ident}.')
    mapping[ident]=value


def read_model(root):
    root=Path(root)
    cameras,images,points={},{},{}
    for line in lines(root/'cameras.txt'):
        if not line.strip() or line.lstrip().startswith('#'): continue
        fields=line.split()
        if len(fields)<5: raise ValueError('Invalid cameras.txt row.')
        ident=nonnegative(fields[0],'camera ID')
        width,height=int(fields[2]),int(fields[3])
        if width<=0 or height<=0: raise ValueError('Camera dimensions must be positive.')
        insert(cameras,ident,dict(model=fields[1],width=width,height=height,params=floats(fields[4:])), 'camera')
    raw=lines(root/'images.txt'); cursor=0
    while cursor<len(raw):
        line=raw[cursor]; cursor+=1
        if not line.strip() or line.lstrip().startswith('#'): continue
        fields=line.split(maxsplit=9)
        if len(fields)!=10: raise ValueError('Image pose requires ID, quaternion, translation, camera ID and filename.')
        ident=nonnegative(fields[0],'image ID')
        while cursor<len(raw) and raw[cursor].lstrip().startswith('#'): cursor+=1
        if cursor>=len(raw): raise ValueError('Missing image observation line, including an empty line for zero observations.')
        observations=raw[cursor].split(); cursor+=1
        if len(observations)%3: raise ValueError('Image observations must be x,y,point3D_id triples.')
        values=[]
        for index in range(0,len(observations),3):
            point_id=int(observations[index+2])
            if point_id < -1: raise ValueError('Only -1 is a valid unassociated point ID.')
            values.append((*floats(observations[index:index+2]),point_id))
        insert(images,ident,dict(q=floats(fields[1:5]),t=floats(fields[5:8]),
                                 camera_id=nonnegative(fields[8],'camera ID'),name=fields[9],observations=values),'image')
    for line in lines(root/'points3D.txt'):
        if not line.strip() or line.lstrip().startswith('#'): continue
        fields=line.split()
        if len(fields)<8 or (len(fields)-8)%2: raise ValueError('Invalid points3D row or track pairs.')
        ident=nonnegative(fields[0],'point ID')
        color=[int(value) for value in fields[4:7]]
        if any(not 0<=value<=255 for value in color): raise ValueError('RGB values must be in 0..255.')
        track=[(nonnegative(fields[i],'track image ID'),nonnegative(fields[i+1],'point2D index')) for i in range(8,len(fields),2)]
        insert(points,ident,dict(xyz=floats(fields[1:4]),rgb=color,error=floats(fields[7:8])[0],track=track),'point')
    return {'cameras':cameras,'images':images,'points':points}
