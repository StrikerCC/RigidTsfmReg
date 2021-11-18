import json

face_points_cam = {
    '右耳上': [220.45, 120.824, 160.298],
    '右耳中': [210.383, 128.495, 132.645],
    '右耳下': [212.443, 132.041, 107.708],

    '左耳上': [55.1835, 124.231, 158.073],
    '左耳中': [62.9985, 129.299, 132.17],
    '左耳下': [55.8734, 135.427, 109.217],

    '右外眼角': [180.126, 213.697, 153.049],
    '右内眼角': [154.419, 219.463, 150.789],

    '左外眼角': [89.975, 210.866, 152.33],
    '左内眼角': [113.841, 218.907, 150.194],

    '左嘴角': [110.566, 214.806, 88.8482],
    '右嘴角': [155.522, 219.27, 90.1044],

    '左鼻根': [116.985, 220.823, 111.095],
    '右鼻根': [153.563, 222.441, 113.083],
    '鼻尖': [134.659, 239.56, 122.295],
}

face_points_ct = {
    "右耳上": [16.93, 71.5, 104.3],
    # "右耳中": [],
    "右耳下": [20.31, 71.91, 155.01],

    "左耳上": [189.31, 58.09, 114.01],
    # "左耳中": [],
    "左耳下": [184.44, 60.53, 164.8],

    "右外眼角": [68.25, 160.14, 135.1],
    "右内眼角": [94.25, 164.13, 141.24],

    "左外眼角": [151.99, 151.13, 141.4],
    "左内眼角": [129.52, 157.63, 143.5],

    "左嘴角": [130.0, 139.34, 201.64],
    "右嘴角": [84.53, 145.03, 198.8],

    "左鼻根": [121.88, 155.94, 183.4],
    "右鼻根": [95.06, 157.63, 180.31],
    "鼻尖": [110.69, 175.52, 173.6],
    "鼻人中": [109.69, 160.88, 183.65],
}


def main():
    f_cam = open('./facepoints_cam.json', 'w')
    f_ct = open('./facepoints_ct.json', 'w')

    json.dump(face_points_cam, f_cam)
    json.dump(face_points_ct, f_ct)


if __name__ == '__main__':
    main()
