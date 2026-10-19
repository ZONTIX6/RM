import argparse
import glob
import os
import sys

import bpy


def material(name, color):
    mat = bpy.data.materials.new(name)
    mat.use_nodes = True
    bsdf = mat.node_tree.nodes.get("Principled BSDF")
    bsdf.inputs["Base Color"].default_value = color
    bsdf.inputs["Roughness"].default_value = 0.55
    bsdf.inputs["Metallic"].default_value = color[3] if False else 0.25
    return mat


def choose_material(filename, mats):
    name = filename.lower()
    if any(k in filename for k in ["2006", "4310", "海泰电机", "GM6020", "g43"]):
        return mats["motor"]
    if any(k in filename for k in ["带轮", "同步带"]):
        return mats["belt"]
    if any(k in filename for k in ["齿轮"]):
        return mats["gear"]
    if any(k in filename for k in ["link", "侧板", "正板", "连接板", "固定板", "护板"]):
        return mats["plate"]
    if any(k in filename for k in ["吸盘"]):
        return mats["rubber"]
    if any(k in filename for k in ["轴承", "轴", "法兰"]):
        return mats["alloy"]
    if any(k in name for k in ["screw", "螺丝", "螺钉", "螺母"]):
        return mats["dark"]
    return mats["default"]


def main():
    argv = sys.argv
    if "--" in argv:
        argv = argv[argv.index("--") + 1:]
    else:
        argv = []

    parser = argparse.ArgumentParser()
    parser.add_argument("--input-dir", required=True)
    parser.add_argument("--prefix", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--skip-fasteners", action="store_true")
    args = parser.parse_args(argv)

    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete()

    mats = {
        "default": material("dark composite", (0.16, 0.18, 0.19, 1.0)),
        "plate": material("dark grey plate", (0.32, 0.37, 0.39, 1.0)),
        "alloy": material("machined alloy", (0.72, 0.75, 0.73, 1.0)),
        "motor": material("black motor housing", (0.03, 0.035, 0.04, 1.0)),
        "belt": material("black belt pulley", (0.02, 0.018, 0.015, 1.0)),
        "gear": material("brass gear", (0.72, 0.55, 0.24, 1.0)),
        "rubber": material("rubber suction cup", (0.01, 0.012, 0.013, 1.0)),
        "dark": material("dark fastener", (0.025, 0.026, 0.028, 1.0)),
    }

    files = sorted(glob.glob(os.path.join(args.input_dir, f"{args.prefix}*.STL")))
    if args.skip_fasteners:
        fastener_words = ["螺丝", "螺钉", "螺母", "垫片", "钢柱", "铝柱"]
        files = [f for f in files if not any(w in os.path.basename(f) for w in fastener_words)]

    for path in files:
        before = set(bpy.data.objects)
        bpy.ops.wm.stl_import(filepath=path)
        after = [obj for obj in bpy.data.objects if obj not in before]
        for obj in after:
            obj.name = os.path.splitext(os.path.basename(path))[0]
            obj.data.name = obj.name
            obj.data.materials.append(choose_material(obj.name, mats))
            obj.scale = (0.001, 0.001, 0.001)

    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)

    objs = [obj for obj in bpy.context.scene.objects if obj.type == "MESH"]
    if not objs:
        raise RuntimeError("No STL meshes imported")

    min_x = min(obj.bound_box[i][0] + obj.location.x for obj in objs for i in range(8))
    max_x = max(obj.bound_box[i][0] + obj.location.x for obj in objs for i in range(8))
    min_y = min(obj.bound_box[i][1] + obj.location.y for obj in objs for i in range(8))
    max_y = max(obj.bound_box[i][1] + obj.location.y for obj in objs for i in range(8))
    min_z = min(obj.bound_box[i][2] + obj.location.z for obj in objs for i in range(8))

    cx = (min_x + max_x) * 0.5
    cy = (min_y + max_y) * 0.5
    for obj in objs:
        obj.location.x -= cx
        obj.location.y -= cy
        obj.location.z -= min_z

    os.makedirs(os.path.dirname(args.output), exist_ok=True)
    bpy.ops.export_scene.gltf(
        filepath=args.output,
        export_format="GLB",
        export_yup=True,
        export_apply=True,
        export_materials="EXPORT",
    )
    print(f"exported={args.output}")
    print(f"objects={len(objs)}")


if __name__ == "__main__":
    main()
