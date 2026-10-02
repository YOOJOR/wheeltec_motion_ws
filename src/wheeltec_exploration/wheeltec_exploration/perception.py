"""Optional local YOLO-World/MobileSAM. Explicit local files; no VLM/cloud/downloads."""
from pathlib import Path
from unittest.mock import patch
import numpy as np
from .core import unproject


def target_points(posed, mask, stride=4):
    mask = np.asarray(mask)
    depth = posed.observation.depth_m
    if mask.shape != depth.shape: raise ValueError('TARGET_MASK_SHAPE_MISMATCH')
    selected = np.where(mask.astype(bool),depth,np.nan)
    points = unproject(selected,np.asarray(posed.observation.depth_info.k).reshape(3,3),stride)
    return posed.camera_points_to_world(points)


class LocalPerception:
    def __init__(self, directory, target='chair', device='cuda', confidence=.55):
        if not target.strip() or not 0<confidence<=1: raise ValueError('INVALID_TARGET_CONFIG')
        directory = Path(directory)
        required = [directory/'yolov8l-world.pt',directory/'mobile_sam.pt',directory/'clip/ViT-B-32.pt']
        if not all(p.is_file() for p in required): raise ValueError('LOCAL_MODELS_MISSING')
        import clip
        from ultralytics import YOLO,SAM
        self.target,self.device,self.confidence=target,device,confidence
        original=clip.load
        def local_load(name,*args,**kwargs):
            if name != 'ViT-B/32': raise ValueError('UNEXPECTED_CLIP_MODEL')
            return original(str(required[2]),*args,**kwargs)
        with patch('clip.load',local_load):
            self.world=YOLO(str(required[0])).to(device)
            self.world.set_classes([target])
        self.sam=SAM(str(required[1])).to(device)

    def detect(self, posed):
        import torch
        image=np.ascontiguousarray(posed.observation.rgb[...,::-1])
        with torch.no_grad():
            result=self.world.predict(image,conf=self.confidence,imgsz=320,
                                      device=self.device,verbose=False)[0]
            if not len(result.boxes): return None, {'target':self.target,'detections':0}
            index=int(result.boxes.conf.argmax())
            box=result.boxes.xyxy[index:index+1]
            segmented=self.sam.predict(image,bboxes=box,device=self.device,verbose=False)[0]
        if segmented.masks is None: return None,{'target':self.target,'detections':len(result.boxes),'usable_masks':0}
        mask=segmented.masks.data[0].cpu().numpy()
        if mask.shape != posed.observation.depth_m.shape:
            import cv2
            mask=cv2.resize(mask,(image.shape[1],image.shape[0]),interpolation=cv2.INTER_NEAREST)
        points=target_points(posed,mask>.5)
        return (points if len(points)>=3 else None),{
            'target':self.target,'detections':len(result.boxes),
            'confidence':float(result.boxes.conf[index].item()),'target_depth_points':len(points)}
