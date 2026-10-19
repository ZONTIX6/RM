# CAD 导出与预览工具

从 `basic_framework` 目录执行以下示例，输入模型和输出文件由使用者提供。
这些工具不依赖 MuJoCo；SolidWorks 脚本需要 Windows 和已安装的 SolidWorks，
Blender 脚本使用 Blender 自带的 Python / `bpy`，不通过普通 pip 安装。

## SolidWorks → STL

```powershell
powershell -ExecutionPolicy Bypass -File tools/export_solidworks_mesh.ps1 -InputPath "C:/models/arm.SLDASM" -OutputPath "C:/exports/arm.stl"
cscript tools/export_solidworks_mesh.vbs "C:/models/arm.SLDASM" "C:/exports/arm.stl"
```

两份脚本为替代入口，按本机 SolidWorks COM 接口兼容情况选择一份执行。

## STL 零件集合 → GLB

```sh
blender --background --python tools/convert_stl_set_to_glb.py -- --input-dir "C:/exports" --prefix "arm" --output "C:/exports/arm.glb" --skip-fasteners
```

转换器按文件名前缀选择零件，按毫米到米缩放，并根据名称分配预览材质。
需要保留紧固件时去掉 `--skip-fasteners`。

## GLB → 预览图片

```sh
blender --background --python tools/render_glb_preview.py -- --input "C:/exports/arm.glb" --output "C:/exports/arm.png"
```

原始 CAD 和生成的模型文件未收录。脚本保留原实现；本次发布只做 Python 语法检查，
没有运行需要 SolidWorks/Blender 的转换流程。
