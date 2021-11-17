
import json

import SimpleITK
import numpy as np
import open3d
import vedo
import vedo.applications
from vedo.mesh import merge


def read_log(file_path):
    f = open(file_path)
    result = json.load(f)
    assert 'tsfm' in result.keys()
    return result


def to_mhd(file_path):
    if file_path[:-3] == 'mhd':
        return file_path
    img = SimpleITK.ReadImage(file_path)
    spacing = img.GetSpacing()
    array = SimpleITK.GetArrayFromImage(img)

    output_file = SimpleITK.GetImageFromArray(array)
    output_file.SetSpacing(spacing)
    SimpleITK.WriteImage(output_file, file_path[:-3] + 'mhd')
    return file_path[:-3] + 'mhd'


# def vis_mhd(file_path):
#     vol = vedo.load(file_path)
#     vedo.show(vol)


def vis(reg_result, face_src, face_tgt):
    src = reg_result['src']
    tgt = reg_result['tgt']
    tsfm = np.asarray(reg_result['tsfm'])

    print('src', src)
    print('tgt', tgt)
    print('tsfm', tsfm)

    # draw points
    origin = vedo.Axes(xrange=(0, 100), yrange=(0, 100), zrange=(0, 100),
                       xtitle='front', ytitle='left', ztitle='head',
                       yzGrid=False, xTitleSize=0.15, yTitleSize=0.15, zTitleSize=0.15,
                       xLabelSize=0, yLabelSize=0, zLabelSize=0, tipSize=0.05,
                       axesLineWidth=2, xLineColor='dr', yLineColor='dg', zLineColor='db',
                       xTitleOffset=0.05, yTitleOffset=0.05, zTitleOffset=0.05,)

    pc_src = vedo.Points(src, r=18, c='g')
    pc_tgt = vedo.Points(tgt, r=18, c='r')

    arrow = vedo.Arrows(pc_src, pc_tgt, s=0.5, alpha=0.2)       # draw line between correspondence

    '''before registration'''
    vedo.show(pc_src, face_src, origin, title='src marker')
    vedo.show(pc_tgt, face_tgt, origin, title='tgt marker')
    vedo.show(pc_src, pc_tgt, arrow, face_src, origin, title='src and tgt match')

    '''after registration'''
    # apply reg result
    pc_src.applyTransform(tsfm)
    face_src = face_src.applyTransform(tsfm)
    arrow = vedo.Arrows(pc_src, pc_tgt, s=1.0, alpha=0.2)

    vedo.show(pc_src, pc_tgt, arrow, title='marker match')
    vedo.show(pc_src, pc_tgt, arrow, face_src, face_tgt, title='face match')

    return


def take_surface(file_path, threshold=[-196.294]):
    if isinstance(file_path, str):
        slice = vedo.load(file_path)
    else:
        slice = file_path
    isos = slice.isosurface(threshold=threshold)
    splitems = isos.splitByConnectivity(maxdepth=5)
    vedo.show(splitems[0])
    return splitems[0]

    # vedo.show(isos, title=file_path)
    # return isos



def vis_reg_result():
    """"""
    ct_file_path = './bin/data/model_man/b.mha'
    cam_file_path = './bin/data/model_man/img0.5.mha'
    ''''''

    ct_file_path = to_mhd(ct_file_path)
    cam_file_path = to_mhd(cam_file_path)

    '''read reg log file'''
    log_file_path = './bin/log.json'
    reg_result = read_log(log_file_path)

    '''read image'''
    face_src = take_surface(ct_file_path, [-196.294])
    face_tgt = take_surface(cam_file_path, [8000.0])

    '''vis'''

    vis(reg_result, face_src, face_tgt)

    return


def main():
    key_pts_file_path = './config/facepoints.json'
    ct_file_path = './bin/data/model_man/brain.mhd'
    dicom_apth = '/home/cheng/proj/3d/TEASER-plusplus/data/human_models/head_models/model_man/722brain'
    f = open(key_pts_file_path, 'r')
    key_pts = json.load(f)

    '''read image'''
    vol = vedo.load(dicom_apth)

    # path = dicom_apth
    # reader = SimpleITK.ImageSeriesReader()
    # dicom_names = reader.GetGDCMSeriesFileNames(path)
    # reader.SetFileNames(dicom_names)
    # image = reader.Execute()
    # SimpleITK.WriteImage(image, './b.mhd')
    # vol = vedo.load('./b.mhd')

    face_src = take_surface(vol, [-196.294])
    key_pc = vedo.Points(list(key_pts.values()), r=15, c='r')
    vedo.show(key_pc, face_src)
    for key in key_pts.keys():
        print(key, ': ', key_pts[key])

        if len(key_pts[key]) < 3:
            continue

        marker = vedo.Point(pos=key_pts[key], r=10, c='r')
        # vedo.show(marker, title=key)
        vedo.show(face_src, marker, title=key)


if __name__ == '__main__':
    main()
