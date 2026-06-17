"""
AprilTag detection + 6-DoF pose estimation for the FISHEYE RGB camera.

Why this is not just "plug fx,fy,cx,cy into a tag detector":
  AprilTag pose solvers (pupil_apriltags, OpenCV solvePnP, etc.) assume a
  PINHOLE camera with NO distortion. Our lens is a strong fisheye (k1..k4).
  Feeding raw fisheye frames + the fisheye K to a pinhole pose solver gives a
  wrong, badly-biased pose -- worst at the image edges where tags often sit.

  Correct recipe (this script):
    1. Undistort each frame to an equivalent PINHOLE image using the fisheye
       intrinsics (cv2.fisheye.initUndistortRectifyMap -> remap). This yields a
       new pinhole matrix P (fx',fy',cx',cy') with ZERO distortion.
    2. Detect AprilTags on the undistorted image (cv2.aruco, APRILTAG dict).
    3. solvePnP with P and distCoeffs=0, using the tag's known physical size.
       -> rvec/tvec give tag pose in the camera frame.

  This keeps everything in one consistent pinhole model. (A faster alternative
  for embedded use is to detect on the raw frame and undistort only the 4 tag
  corners with cv2.fisheye.undistortPoints(P=P) before solvePnP -- see the
  pose_from_corners() docstring. The full-image route below is more robust.)

Dependencies: only OpenCV (cv2.aruco ships the AprilTag dictionaries). No extra
pip install needed. Detector family default: tag36h11.

Usage:
    # 离线图片模式
    python apriltag_pose.py <image_or_folder> --tag-size 0.06 [--family 36h11]
                                              [--balance 0.0] [--show]

    # 实时摄像头模式（按 q 退出）
    python apriltag_pose.py --live [--camera 0] --tag-size 0.06 [--family 36h11]
                                   [--balance 0.0] [--show]

    --tag-size : physical edge length of the black tag border, in METRES.
                 (pose translation is reported in the same unit)
"""
import argparse
import sys
import time
from pathlib import Path
import cv2
import numpy as np

ROOT = Path(__file__).resolve().parent
NPZ = ROOT / "data" / "rgb_intrinsics.npz"

_FAMILY = {
    "36h11": cv2.aruco.DICT_APRILTAG_36h11,
    "25h9":  cv2.aruco.DICT_APRILTAG_25h9,
    "16h5":  cv2.aruco.DICT_APRILTAG_16h5,
}


def load_intrinsics():
    d = np.load(str(NPZ))
    return (d["K"].astype(np.float64),
            d["D"].astype(np.float64).reshape(4, 1),
            tuple(int(x) for x in d["image_size"]))


def make_undistorter(K, D, size, balance):
    w, h = size
    P = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
        K, D, (w, h), np.eye(3), balance=float(balance))
    m1, m2 = cv2.fisheye.initUndistortRectifyMap(
        K, D, np.eye(3), P, (w, h), cv2.CV_16SC2)
    return P, m1, m2


def tag_object_points(tag_size):
    """Tag corner coordinates in the tag's own frame (Z=0 plane), in the same
    order cv2.aruco returns image corners: top-left, top-right, bottom-right,
    bottom-left. Centre of the tag is the origin."""
    h = tag_size / 2.0
    return np.array([[-h,  h, 0.0],
                     [ h,  h, 0.0],
                     [ h, -h, 0.0],
                     [-h, -h, 0.0]], dtype=np.float64)


def pose_from_corners(corners_px, tag_size, P):
    """solvePnP a single tag. corners_px: (4,2) pixel corners on the PINHOLE
    (undistorted) image; P: pinhole matrix; distortion is zero by construction.

    Embedded alternative without full-image undistort:
        und = cv2.fisheye.undistortPoints(raw_corners.reshape(-1,1,2), K, D, P=P)
        then pass und.reshape(4,2) here as corners_px.
    """
    objp = tag_object_points(tag_size)
    ok, rvec, tvec = cv2.solvePnP(
        objp, corners_px.astype(np.float64), P, None,
        flags=cv2.SOLVEPNP_IPPE_SQUARE)
    return (rvec, tvec) if ok else (None, None)


def detect_and_print(frame, detector, tag_size, K, D, size, P, m1, m2, balance, label="", no_undistort=False):
    """去畸变 -> 检测 -> 打印位姿。返回可视化图像和检测到的 tag 数量。"""
    h, w = frame.shape[:2]
    if no_undistort:
        und = frame
        Pf = K
    elif (w, h) == size:
        und = cv2.remap(frame, m1, m2, cv2.INTER_LINEAR)
        Pf = P
    else:
        Pf, mm1, mm2 = make_undistorter(K, D, (w, h), balance)
        und = cv2.remap(frame, mm1, mm2, cv2.INTER_LINEAR)

    gray = cv2.cvtColor(und, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = detector.detectMarkers(gray)

    vis = und.copy()
    if ids is None or len(ids) == 0:
        if label:
            print(f"{label}: no AprilTag detected")
        return vis, 0

    cv2.aruco.drawDetectedMarkers(vis, corners, ids)
    prefix = f"{label}: " if label else ""
    print(f"{prefix}{len(ids)} tag(s)")
    for tag_corners, tag_id in zip(corners, ids.ravel()):
        cpx = tag_corners.reshape(4, 2)
        rvec, tvec = pose_from_corners(cpx, tag_size, Pf)
        if rvec is None:
            print(f"  id={tag_id}: pose failed")
            continue
        t = tvec.ravel()
        dist = float(np.linalg.norm(t))
        print(f"  id={tag_id}: t(m)=[{t[0]:+.3f} {t[1]:+.3f} {t[2]:+.3f}] "
              f"range={dist:.3f}m  rvec={rvec.ravel().round(3)}")
        cv2.drawFrameAxes(vis, Pf, None, rvec, tvec, tag_size * 0.5)

    return vis, len(ids)


def run_live(args, K, D, size, P, m1, m2, detector):
    cap = cv2.VideoCapture(args.camera)
    if not cap.isOpened():
        sys.exit(f"cannot open camera {args.camera}")

    # Intrinsics were calibrated at `size`; force the camera to stream at the
    # same resolution. Otherwise K/D don't match the frames and pose is garbage.
    cw, ch = size
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, cw)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, ch)
    got = (int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
           int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
    if got != size:
        print(f"WARNING: camera streams {got}, intrinsics are for {size}. "
              f"Pose will be biased -- recalibrate at {got} or force {size}.")

    print(f"live mode: camera {args.camera} @ {got}  |  press q to quit")
    frame_idx = 0
    t0 = time.time()

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                print("frame read failed, retrying...")
                continue

            frame_idx += 1
            elapsed = time.time() - t0
            fps = frame_idx / elapsed if elapsed > 0 else 0.0

            vis, n = detect_and_print(
                frame, detector, args.tag_size,
                K, D, size, P, m1, m2, args.balance,
                label=f"frame {frame_idx:05d} ({fps:.1f} fps)",
                no_undistort=args.no_undistort)

            if args.show:
                cv2.imshow("apriltag live", vis)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
    except KeyboardInterrupt:
        print("\nstopped by user")
    finally:
        cap.release()
        if args.show:
            cv2.destroyAllWindows()


def run_files(args, K, D, size, P, m1, m2, detector):
    src = Path(args.path)
    files = [src] if src.is_file() else sorted(
        p for p in src.iterdir() if p.suffix.lower() in {".png", ".jpg", ".jpeg", ".bmp"})
    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)

    for f in files:
        img = cv2.imread(str(f))
        if img is None:
            print(f"skip (unreadable): {f}")
            continue

        vis, n = detect_and_print(
            img, detector, args.tag_size,
            K, D, size, P, m1, m2, args.balance,
            label=f"\n{f.name}",
            no_undistort=args.no_undistort)

        out = out_dir / f"{f.stem}_tags.png"
        cv2.imwrite(str(out), vis)
        print(f"  saved: {out}")
        if args.show:
            cv2.imshow("apriltag", vis)
            cv2.waitKey(0)

    if args.show:
        cv2.destroyAllWindows()


def main():
    ap = argparse.ArgumentParser()
    mode = ap.add_mutually_exclusive_group()
    mode.add_argument("path", nargs="?", help="image file or folder (offline mode)")
    mode.add_argument("--live", action="store_true", help="real-time camera mode")
    ap.add_argument("--camera", type=int, default=0,
                    help="camera index for --live (default 0)")
    # tag-size = OUTER black-border edge of the tag (what solvePnP needs), NOT
    # one module. tag36h11 = 8 modules across (6 data + 1 black border/side).
    # Our printed tags: 35 mm per module -> 8 * 0.035 = 0.28 m.
    ap.add_argument("--tag-size", type=float, default=0.28,
                    help="tag black-border edge length in metres "
                         "(default 0.28 = 8 modules * 35 mm)")
    ap.add_argument("--family", default="36h11", choices=list(_FAMILY))
    ap.add_argument("--balance", type=float, default=0.0)
    # This lens IS fisheye (intrinsics model=fisheye, 4 distortion coeffs), so
    # undistortion MUST run by default. The old code hard-set default=True here,
    # which made the undistort path dead -- raw fisheye frames went straight to
    # the pinhole solvePnP and the pose was biased (worst at image edges).
    ap.add_argument("--no-undistort", action="store_true",
                    help="skip fisheye undistortion (NOT recommended: lens is fisheye)")
    ap.add_argument("--show", action="store_true")
    ap.add_argument("--out", default=str(ROOT / "data" / "apriltag"))
    args = ap.parse_args()

    if not args.live and args.path is None:
        ap.error("specify a path (offline) or --live (camera)")

    K, D, size = load_intrinsics()
    P, m1, m2 = make_undistorter(K, D, size, args.balance)

    aruco_dict = cv2.aruco.getPredefinedDictionary(_FAMILY[args.family])
    params = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, params)

    if args.live:
        run_live(args, K, D, size, P, m1, m2, detector)
    else:
        run_files(args, K, D, size, P, m1, m2, detector)


if __name__ == "__main__":
    main()
