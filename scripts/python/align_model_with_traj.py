import os
from io_tools import *

def colmap_model_aligner(input_model_path, ref_images_path, output_model_path):
    random_seed = str(1)
    min_common_images = str(3)
    robust_alignment = str(1)
    # 15cm是pointcloud filter的空间分辨率
    robust_alignment_max_error = str(0.15)
    full_commant_str = '/home/alex/Documents/tools/colmap/build/src/exe/colmap model_aligner --random_seed ' + random_seed + \
        ' --input_path '+ input_model_path + ' --ref_images_path ' + ref_images_path + ' --output_path ' + output_model_path + \
        ' --min_common_images ' + min_common_images + ' --robust_alignment ' + robust_alignment + \
        ' --robust_alignment_max_error ' + robust_alignment_max_error
    print(full_commant_str)
    os.system(full_commant_str)
    return


if __name__ == '__main__':

    #需要输入的参数
    input_model_path = '/home/alex/Desktop/Lidar_Visual_Plane/0603_long_try/sparse_raw'
    output_model_path = '/home/alex/Desktop/Lidar_Visual_Plane/0603_long_try/sparse_aligned'
    ref_image_geo_path = '/home/alex/Desktop/Lidar_Visual_Plane/0603_long_try/geos.txt'
    mkdir(output_model_path)
    colmap_model_aligner(input_model_path,ref_image_geo_path,output_model_path)