import os
import subprocess

from ament_index_python.packages import get_package_share_directory

def main():
    output_path = os.path.dirname(os.path.abspath(__file__))
    urdf_description_file = os.path.join(output_path, "..", "xacro", "dynaarm_standalone.urdf.xacro")

    command = f"xacro -o {output_path}/dynaarm_standalone.urdf {urdf_description_file}"
    print(f'executing command {command}')
    try:
        _ = subprocess.run(command, shell=True, capture_output=True, encoding="utf-8")
    except subprocess.CalledProcessError as e:
        print(f'Error executing command {command}: {e.stderr}')


if __name__ == "__main__":
    main()