# -*- coding: utf-8 -*-
import os
import numpy as np
import yaml
import math


def mkmat(rows, cols, L):
    mat = np.matrix(L, dtype='float64')
    mat.resize((rows,cols))
    return mat

# A yaml constructor is for loading from a yaml node.
# This is taken from @misha 's answer: http://stackoverflow.com/a/15942429
def opencv_matrix_constructor(loader, node):
    mapping = loader.construct_mapping(node, deep=True)
    mat = np.array(mapping["data"])
    if mapping["cols"] > 1:
        mat.resize(mapping["rows"], mapping["cols"])
    else:
        mat.resize(mapping["rows"], )
    return mat
yaml.add_constructor(u"tag:yaml.org,2002:opencv-matrix", opencv_matrix_constructor)


# A yaml representer is for dumping structs into a yaml node.
# So for an opencv_matrix type (to be compatible with c++'s FileStorage) we save the rows, cols, type and flattened-data
def opencv_matrix_representer(dumper, mat):
    if mat.ndim > 1:
        mapping = {'rows': mat.shape[0], 'cols': mat.shape[1], 'dt': 'd', 'data': mat.reshape(-1).tolist()}
    else:
        mapping = {'rows': mat.shape[0], 'cols': 1, 'dt': 'd', 'data': mat.tolist()}
    return dumper.represent_mapping(u"tag:yaml.org,2002:opencv-matrix", mapping)
yaml.add_representer(np.ndarray, opencv_matrix_representer)

def readYAMLFile(fileName):
    ret = {}
    skip_or_not = False
    skip_lines=1    # Skip the first line which says "%YAML:1.0". Or replace it with "%YAML 1.0"
    with open(fileName) as fin:
        for _ in range(skip_lines):
            line = fin.readline()
        if (line[0]=='%'):
            skip_or_not = True
    if (skip_or_not):
        with open(fileName) as fin:
            for i in range(skip_lines):
                line = fin.readline()
            yamlFileOut = fin.read()
    else:
        with open(fileName) as fin:
            yamlFileOut = fin.read()
    # myRe = re.compile(r":([^ ])")   # Add space after ":", if it doesn't exist. Python yaml requirement
    # yamlFileOut = myRe.sub(r': \1', yamlFileOut)
    ret = yaml.load(yamlFileOut, Loader=yaml.FullLoader)
    return ret

def writeYAMLFile(fileName, dict_obj):
    with open(fileName, 'w') as f:
        yaml.dump(dict_obj, f)

def cameraCalibrationYamlBuf(d, k, r, p, image_shape, name="camera"):
    calmessage = (""
    + "image_width: " + str(image_shape[1]) + "\n"
    + "image_height: " + str(image_shape[0]) + "\n"
    + "camera_name: " + name + "\n"
    + "camera_matrix:\n"
    + "  rows: 3\n"
    + "  cols: 3\n"
    + "  dt: d\n"
    + "  data: [" + ", ".join(["%8f" % i for i in k.reshape(1,9)[0]]) + "]\n"
    + "distortion_model: " + ("rational_polynomial" if d.size > 5 else "plumb_bob") + "\n"
    + "distortion_coefficients:\n"
    + "  rows: 1\n"
    + "  cols: 5\n"
    + "  dt: d\n"
    + "  data: [" + ", ".join(["%8f" % i for i in np.squeeze(d)]) + "]\n"
    + "rectification_matrix:\n"
    + "  rows: 3\n"
    + "  cols: 3\n"
    + "  dt: d\n"
    + "  data: [" + ", ".join(["%8f" % i for i in r.reshape(1,9)[0]]) + "]\n"
    + "projection_matrix:\n"
    + "  rows: 3\n"
    + "  cols: 4\n"
    + "  dt: d\n"
    + "  data: [" + ", ".join(["%8f" % i for i in p.reshape(1,12)[0]]) + "]\n"
    + "")
    return calmessage

def lidarCalibrationYamlBuf(R, T, euler, name="lidar"):
    print(R)
    print(T)
    print(euler)
    calmessage = (""
    + "lidar_name: " + name + "\n"
    + "R:\n"
    + "  rows: 3\n"
    + "  cols: 3\n"
    + "  dt: d\n"
    + "  data: [" + ", ".join(["%8f" % i for i in R.reshape(1,9)[0]]) + "]\n"
    + "T:\n"
    + "  rows: 3\n"
    + "  cols: 1\n"
    + "  dt: d\n"
    + "  data: [" + ", ".join(["%8f" % i for i in T.reshape(3,1)]) + "]\n"
    + "euler:\n"
    + "  rows: 1\n"
    + "  cols: 3\n"
    + "  dt: d\n"
    + "  data: [" + ", ".join(["%8f" % i for i in np.squeeze(euler)]) + "]\n"
    + "")
    return calmessage

def cameraLidarCalibrationYamlBuf(R, T, camera_matrix):
    print(R)
    print(T)
    print(camera_matrix)
    calmessage = (""
    + "RT:\n"
    + "  rows: 4\n"
    + "  cols: 4\n"
    + "  dt: d\n"
    + "  data: [" + ", ".join(["%8f" % i for i in np.row_stack((np.column_stack((R, T.reshape(-1,1))), np.array([[0,0,0,1]]))).reshape(1,16)[0]]) + "]\n"
    + "INTRINS:\n"
    + "  rows: 3\n"
    + "  cols: 4\n"
    + "  dt: d\n"
    + "  data: [" + ", ".join(["%8f" % i for i in np.column_stack((camera_matrix, np.zeros((3,1)))).reshape(1,12)[0]]) + "]\n"
    + "")
    return calmessage


# epsilon for testing whether a number is close to zero
_EPS = np.finfo(float).eps * 4.0

# axis sequences for Euler angles
_NEXT_AXIS = [1, 2, 0, 1]

# map axes strings to/from tuples of inner axis, parity, repetition, frame
_AXES2TUPLE = {
    'sxyz': (0, 0, 0, 0), 'sxyx': (0, 0, 1, 0), 'sxzy': (0, 1, 0, 0),
    'sxzx': (0, 1, 1, 0), 'syzx': (1, 0, 0, 0), 'syzy': (1, 0, 1, 0),
    'syxz': (1, 1, 0, 0), 'syxy': (1, 1, 1, 0), 'szxy': (2, 0, 0, 0),
    'szxz': (2, 0, 1, 0), 'szyx': (2, 1, 0, 0), 'szyz': (2, 1, 1, 0),
    'rzyx': (0, 0, 0, 1), 'rxyx': (0, 0, 1, 1), 'ryzx': (0, 1, 0, 1),
    'rxzx': (0, 1, 1, 1), 'rxzy': (1, 0, 0, 1), 'ryzy': (1, 0, 1, 1),
    'rzxy': (1, 1, 0, 1), 'ryxy': (1, 1, 1, 1), 'ryxz': (2, 0, 0, 1),
    'rzxz': (2, 0, 1, 1), 'rxyz': (2, 1, 0, 1), 'rzyz': (2, 1, 1, 1)}

_TUPLE2AXES = dict((v, k) for k, v in _AXES2TUPLE.items())

def euler_from_matrix(matrix, axes='sxyz'):
    """Return Euler angles from rotation matrix for specified axis sequence.

    axes : One of 24 axis sequences as string or encoded tuple

    Note that many Euler angle triplets can describe one matrix.

    >>> R0 = euler_matrix(1, 2, 3, 'syxz')
    >>> al, be, ga = euler_from_matrix(R0, 'syxz')
    >>> R1 = euler_matrix(al, be, ga, 'syxz')
    >>> numpy.allclose(R0, R1)
    True
    >>> angles = (4.0*math.pi) * (numpy.random.random(3) - 0.5)
    >>> for axes in _AXES2TUPLE.keys():
    ...    R0 = euler_matrix(axes=axes, *angles)
    ...    R1 = euler_matrix(axes=axes, *euler_from_matrix(R0, axes))
    ...    if not numpy.allclose(R0, R1): print axes, "failed"

    """
    try:
        firstaxis, parity, repetition, frame = _AXES2TUPLE[axes.lower()]
    except (AttributeError, KeyError):
        _ = _TUPLE2AXES[axes]
        firstaxis, parity, repetition, frame = axes

    i = firstaxis
    j = _NEXT_AXIS[i+parity]
    k = _NEXT_AXIS[i-parity+1]

    M = np.array(matrix, dtype=np.float64, copy=False)[:3, :3]
    if repetition:
        sy = math.sqrt(M[i, j]*M[i, j] + M[i, k]*M[i, k])
        if sy > _EPS:
            ax = math.atan2( M[i, j],  M[i, k])
            ay = math.atan2( sy,       M[i, i])
            az = math.atan2( M[j, i], -M[k, i])
        else:
            ax = math.atan2(-M[j, k],  M[j, j])
            ay = math.atan2( sy,       M[i, i])
            az = 0.0
    else:
        cy = math.sqrt(M[i, i]*M[i, i] + M[j, i]*M[j, i])
        if cy > _EPS:
            ax = math.atan2( M[k, j],  M[k, k])
            ay = math.atan2(-M[k, i],  cy)
            az = math.atan2( M[j, i],  M[i, i])
        else:
            ax = math.atan2(-M[j, k],  M[j, j])
            ay = math.atan2(-M[k, i],  cy)
            az = 0.0

    if parity:
        ax, ay, az = -ax, -ay, -az
    if frame:
        ax, az = az, ax
    return ax, ay, az

def rotation_matrix_from_vectors(vec1, vec2):
    """ Find the rotation matrix that aligns vec1 to vec2
    :param vec1: A 3d "source" vector
    :param vec2: A 3d "destination" vector
    :return mat: A transform matrix (3x3) which when applied to vec1, aligns it with vec2.
    """
    a, b = (vec1 / np.linalg.norm(vec1)).reshape(3), (vec2 / np.linalg.norm(vec2)).reshape(3)
    v = np.cross(a, b)
    c = np.dot(a, b)
    s = np.linalg.norm(v)
    kmat = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    rotation_matrix = np.eye(3) + kmat + kmat.dot(kmat) * ((1 - c) / (s ** 2))
    return rotation_matrix


def points2pcd(points, PCD_FILE_PATH):
	# save path
    # PCD_DIR_PATH=os.path.join(os.path.abspath('.'),'pcd')
    # PCD_FILE_PATH=os.path.join(PCD_DIR_PATH,'cache.pcd')
    if os.path.exists(PCD_FILE_PATH):
    	os.remove(PCD_FILE_PATH)
    
    #写文件句柄
    handle = open(PCD_FILE_PATH, 'a')
    
    #得到点云点数
    point_num=points.shape[0]

    #pcd头部（重要）
    if (points.shape[1]==3):
        handle.write('# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1')
    elif (points.shape[1]>3):
        handle.write('# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1')
    string = '\nWIDTH ' + str(point_num)
    handle.write(string)
    handle.write('\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0')
    string = '\nPOINTS ' + str(point_num)
    handle.write(string)
    handle.write('\nDATA ascii')

    # 依次写入点
    for i in range(point_num):
        string = '\n' + ' '.join(map(str, points[i, :4]))
        # string = '\n' + str(points[i, 0]) + ' ' + str(points[i, 1]) + ' ' + str(points[i, 2])
        handle.write(string)
    handle.close()
