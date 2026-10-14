from collections import defaultdict
import math
from pathlib import Path
import statistics
from .common import Findings, integer, number
from .model import PARAMETERS


def rotation(q):
    w,x,y,z=q; norm=math.sqrt(sum(value*value for value in q))
    if norm==0: return None
    w,x,y,z=[v/norm for v in q]
    return [[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
            [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
            [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]]


def audit(model,images_root=None,other=None,min_overlap=3,max_reprojection_px=4.0):
    integer(min_overlap,'min_overlap',1)
    number(max_reprojection_px,'max_reprojection_px')
    if max_reprojection_px<=0: raise ValueError('Reprojection threshold must be positive.')
    findings=Findings(); cameras=model['cameras']; images=model['images']; points=model['points']
    if not images: findings.add('no_registered_images','error','Model has no registered images.')
    if not points: findings.add('no_points','warning','Model has no 3D points; pose-only models can be intentional.')
    valid_cameras=set()
    for ident,camera in cameras.items():
        name=camera['model']; params=camera['params']
        if name not in PARAMETERS:
            findings.add('unvalidated_camera_model','warning','Parameter count and projection for this model are not implemented.',camera_id=ident,model=name)
            continue
        if len(params)!=PARAMETERS[name]:
            findings.add('parameter_count','error','Camera has an incorrect number of parameters.',camera_id=ident,model=name)
            continue
        focal_count=1 if name in {'SIMPLE_PINHOLE','SIMPLE_RADIAL','RADIAL','SIMPLE_RADIAL_FISHEYE','RADIAL_FISHEYE'} else 2
        if any(value<=0 for value in params[:focal_count]):
            findings.add('invalid_focal_length','error','Focal lengths must be positive.',camera_id=ident)
        else: valid_cameras.add(ident)
    track_sets={ident:set(point['track']) for ident,point in points.items()}
    graph=defaultdict(set); reprojections=[]; projected=0; names=defaultdict(list)
    for ident,image in images.items():
        names[image['name']].append(ident)
        if image['camera_id'] not in cameras:
            findings.add('missing_camera','error','Image references an absent camera.',image_id=ident,camera_id=image['camera_id'])
        qnorm=math.sqrt(sum(x*x for x in image['q']))
        if abs(qnorm-1)>1e-3:
            findings.add('quaternion_norm','error','World-to-camera quaternion is not unit length.',image_id=ident,norm=qnorm)
        R=rotation(image['q']) if abs(qnorm-1)<=1e-3 else None
        if images_root:
            relative=Path(image['name'].replace('\\','/'))
            root=Path(images_root).resolve()
            target=(root/relative).resolve()
            if relative.is_absolute() or '..' in relative.parts or not target.is_relative_to(root):
                findings.add('unsafe_image_name','error','Image name is not a safe relative path.',image_id=ident)
            elif not target.is_file():
                findings.add('missing_image_file','error','Referenced image file does not exist.',image_id=ident)
        for index,(x,y,point_id) in enumerate(image['observations']):
            if point_id==-1: continue
            if point_id not in points:
                findings.add('missing_point','error','Observation references an absent 3D point.',image_id=ident,index=index,point_id=point_id)
                continue
            if (ident,index) not in track_sets[point_id]:
                findings.add('missing_reciprocal_track','error','Observation is not present in the corresponding 3D point track.',image_id=ident,index=index,point_id=point_id)
            camera=cameras.get(image['camera_id'])
            if R is None or image['camera_id'] not in valid_cameras or camera['model'] not in {'PINHOLE','SIMPLE_PINHOLE'}: continue
            xyz=points[point_id]['xyz']
            local=[sum(R[row][col]*xyz[col] for col in range(3))+image['t'][row] for row in range(3)]
            projected+=1
            if local[2]<=0:
                findings.add('point_behind_camera','warning','Referenced point is not in front of the pinhole camera.',image_id=ident,point_id=point_id)
                continue
            if camera['model']=='PINHOLE': fx,fy,cx,cy=camera['params']
            else: fx,cx,cy=camera['params']; fy=fx
            error=math.hypot(fx*local[0]/local[2]+cx-x,fy*local[1]/local[2]+cy-y)
            reprojections.append(error)
            if error>max_reprojection_px:
                findings.add('large_pinhole_residual','warning','Computed residual exceeds the diagnostic threshold.',image_id=ident,point_id=point_id,pixels=error)
    for name,ids in names.items():
        if len(ids)>1: findings.add('duplicate_image_name','error','Registered image names are not unique.',image_ids=ids)
    for point_id,point in points.items():
        track=point['track']
        if len(set(track))!=len(track): findings.add('duplicate_track_pair','error','Point track repeats an image/index pair.',point_id=point_id)
        if len({i for i,_ in track})!=len(track): findings.add('multiple_features_same_image','error','Point track contains multiple observations from one image.',point_id=point_id)
        if len(track)<2: findings.add('short_track','warning','Point has fewer than two observations.',point_id=point_id)
        if point['error']<0: findings.add('negative_stored_error','warning','Stored reprojection error is negative/uninitialized.',point_id=point_id)
        valid=[]
        for image_id,index in track:
            image=images.get(image_id)
            if image is None:
                findings.add('missing_track_image','error','Point track references an absent image.',point_id=point_id,image_id=image_id)
            elif index>=len(image['observations']):
                findings.add('track_index_out_of_range','error','Point2D index is outside the image observations.',point_id=point_id,image_id=image_id,index=index)
            elif image['observations'][index][2]!=point_id:
                findings.add('track_points_to_other_id','error','Track observation references a different 3D point.',point_id=point_id,image_id=image_id,index=index)
            else: valid.append(image_id)
        if valid:
            anchor=valid[0]
            for image_id in valid[1:]: graph[anchor].add(image_id); graph[image_id].add(anchor)
    components=[]; unseen=set(images)
    while unseen:
        start=min(unseen); component={start}; pending=[start]
        while pending:
            for nxt in graph[pending.pop()]-component: component.add(nxt); pending.append(nxt)
        unseen-=component; components.append(sorted(component))
    if len(components)>1: findings.add('disconnected_view_graph','warning','Shared-track view graph has multiple components.',count=len(components))
    merge=None
    if other is not None:
        shared=sorted(set(names)&{image['name'] for image in other['images'].values()})
        merge={'shared_names':shared,'count':len(shared),'minimum':min_overlap,'meets_name_threshold':len(shared)>=min_overlap,
               'semantics':'Exact-name candidate overlap only; no geometric alignment or merge guarantee.'}
        if len(shared)<min_overlap: findings.add('insufficient_named_overlap','warning','Named overlap is below the explicit candidate threshold.',count=len(shared))
    return dict(tool='colmap-model-audit',schema_version=1,cameras=len(cameras),registered_images=len(images),points3D=len(points),
                median_track_length=statistics.median([len(p['track']) for p in points.values()]) if points else None,
                view_components=components,pinhole_observations_checked=projected,
                median_pinhole_residual_px=statistics.median(reprojections) if reprojections else None,
                merge_overlap=merge,**findings.report())
