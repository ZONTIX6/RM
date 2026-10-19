import argparse
import math
import os
import sys

import bpy
from mathutils import Vector


def main():
    argv = sys.argv
    if "--" in argv:
        argv = argv[argv.index("--") + 1:]
    else:
        argv = []

    parser = argparse.ArgumentParser()
    parser.add_argument("--input", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args(argv)

    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete()
    bpy.ops.import_scene.gltf(filepath=args.input)

    objs = [obj for obj in bpy.context.scene.objects if obj.type == "MESH"]
    box = None
    for obj in objs:
        obj_box = [obj.matrix_world @ Vector(corner) for corner in obj.bound_box]
        for point in obj_box:
            if box is None:
                box = [point.copy(), point.copy()]
            else:
                box[0].x = min(box[0].x, point.x)
                box[0].y = min(box[0].y, point.y)
                box[0].z = min(box[0].z, point.z)
                box[1].x = max(box[1].x, point.x)
                box[1].y = max(box[1].y, point.y)
                box[1].z = max(box[1].z, point.z)

    center = (box[0] + box[1]) * 0.5
    size = box[1] - box[0]
    max_dim = max(size.x, size.y, size.z)

    light_data = bpy.data.lights.new("Key", "AREA")
    light = bpy.data.objects.new("Key", light_data)
    light.location = (center.x + max_dim, center.y - max_dim, center.z + max_dim)
    light.data.energy = 500
    light.data.size = max_dim
    bpy.context.collection.objects.link(light)

    camera_data = bpy.data.cameras.new("Camera")
    camera = bpy.data.objects.new("Camera", camera_data)
    distance = max_dim * 1.65
    camera.location = (center.x + distance * 0.85, center.y - distance, center.z + distance * 0.55)
    direction = center - camera.location
    camera.rotation_euler = direction.to_track_quat("-Z", "Y").to_euler()
    camera.data.lens = 42
    bpy.context.collection.objects.link(camera)
    bpy.context.scene.camera = camera

    bpy.context.scene.render.resolution_x = 1400
    bpy.context.scene.render.resolution_y = 900
    bpy.context.scene.render.film_transparent = False
    bpy.context.scene.world.color = (0.06, 0.07, 0.08)
    bpy.context.scene.eevee.taa_render_samples = 32

    os.makedirs(os.path.dirname(args.output), exist_ok=True)
    bpy.context.scene.render.filepath = args.output
    bpy.ops.render.render(write_still=True)
    print(f"preview={args.output}")
    print(f"objects={len(objs)}")
    print(f"bbox=({size.x:.3f}, {size.y:.3f}, {size.z:.3f})")


if __name__ == "__main__":
    main()
