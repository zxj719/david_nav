"""把 xCoreSDK_python 所在目录加入 sys.path。
位置：/home/agx/wheelhumanoidrobot/workspace_arm/Arm_skills/Release/...
"""
import os
import sys
import platform

ARM_SKILLS = "/home/agx/wheelhumanoidrobot/workspace_arm/Arm_skills"

def add_release_path():
    release = os.path.join(ARM_SKILLS, "Release")
    linux = os.path.join(release, "linux")
    arm = os.path.join(linux, "arm")
    candidates = [ARM_SKILLS, release, linux, arm,
                  os.path.join(arm, "xCoreSDK_python"),
                  os.path.join(linux, "xCoreSDK_python")]
    for p in candidates:
        if os.path.isdir(p) and p not in sys.path:
            sys.path.insert(0, p)

add_release_path()
