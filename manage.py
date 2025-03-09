#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PIUS単体での手動・推論制御スクリプト

* drive *
PIUSでの使用を前提に、幾つかのクラスを置き換えています。
使用機器も固定するためハードコードが目立ちます。

* train *
教師データからの模倣学習、教師データと既存モデルから転移学習が行えます。

Usage:
    manage.py drive [--model=<model>]
    manage.py train [--tubs=<tub_paths>] (--model=<model>)
    [--transfer=<transfer_model>]
    [--comment=<comment>]

Options:
    -h --help          Show this screen.
"""

# 不具合対策
try:
    import cv2
    import skimage
    import sklearn

    print(f"cv2: {cv2.__version__}")
    print(f"sklearn: {sklearn.__version__}")
    print(f"skimage: {skimage.__version__}")
except Exception:
    pass

import logging
import os
import struct
import time

import serial
from docopt import docopt

import donkeycar as dk
from donkeycar.parts.camera import CSICamera
from donkeycar.parts.datastore import TubHandler
from donkeycar.parts.tub_v2 import TubWriter
from donkeycar.pipeline.augmentations import ImageAugmentation
from donkeycar.pipeline.training import train

logger = logging.getLogger()
logging.basicConfig(level=logging.INFO)


def drive(cfg, model_path=None, model_type=None):
    """
    運転時のメインループ開始前の処理

    Parameters
    ----------
    cfg : dict
        起動時に読み込まれるコンフィグファイル。
    model_path : str, optional
        推論時に使用するモデルへのパス。指定されていないと、AI制御は無効になります。
        by default None.
    """
    logger.info(f"PID: {os.getpid()}")

    # 車両インスタンス
    car = dk.vehicle.Vehicle()
    inputs = []

    # カメラを追加
    if cfg.CAMERA_TYPE == "CSIC":
        cam = CSICamera(
            image_w=cfg.IMAGE_W,
            image_h=cfg.IMAGE_H,
            image_d=cfg.IMAGE_DEPTH,
            framerate=cfg.CAMERA_FRAMERATE,
            capture_width=1920,
            capture_height=1080,
            gstreamer_flip=cfg.CSIC_CAM_GSTREAMER_FLIP_PARM,
        )
        car.add(cam, inputs=inputs, outputs=["cam/image_array"], threaded=True)
    else:
        raise (Exception("Invalid camera type for PIUS: %s" % cfg.CAMERA_TYPE))

    # コントローラを追加
    if cfg.USE_JOYSTICK_AS_DEFAULT:
        from donkeycar.parts.controller import get_js_controller

        ctr = get_js_controller(cfg)
        if cfg.CONTROLLER_TYPE != "ps4":
            raise (
                Exception("Invalid controller type for PIUS: %s" % cfg.CONTROLLER_TYPE)
            )
        if cfg.USE_NETWORKED_JS:
            from donkeycar.parts.controller import JoyStickSub

            netwkJs = JoyStickSub(cfg.NETWORK_JS_SERVER_IP)
            car.add(netwkJs, threaded=True)
            ctr.js = netwkJs
    car.add(
        ctr,
        inputs=["cam/image_array"],
        outputs=["user/angle", "user/throttle", "user/brake", "user/mode", "recording"],
        threaded=True,
    )

    # AI制御への切り替えのための操作設定監視パーツの追加
    car.add(PilotCondition(), inputs=["user/mode"], outputs=["run_pilot"])

    # オートパイロットの追加
    if model_type is None:
        model_type = cfg.DEFAULT_MODEL_TYPE
    if model_type != "linear":
        raise (Exception("Invalid model type for PIUS: %s" % cfg.DEFAULT_MODEL_TYPE))
    if model_path:
        kl = dk.utils.get_model_by_type(model_type, cfg)
        kl.load(model_path=model_path)
        inputs = ["cam/image_array"]
        # Add image transformations like crop or trapezoidal mask
        if hasattr(cfg, "TRANSFORMATIONS") and cfg.TRANSFORMATIONS:
            outputs = ["cam/image_array_trans"]
            car.add(
                ImageAugmentation(cfg, "TRANSFORMATIONS"),
                inputs=inputs,
                outputs=outputs,
            )
            inputs = outputs

        outputs = ["pilot/angle", "pilot/throttle", "pilot/brake"]
        car.add(kl, inputs=inputs, outputs=outputs, run_condition="run_pilot")

    # モード毎に運転に影響を与えるパラメータの設定パーツの追加
    car.add(
        DriveMode(cfg=cfg),
        inputs=[
            "user/mode",
            "user/angle",
            "user/throttle",
            "user/brake",
            "pilot/angle",
            "pilot/throttle",
            "pilot/brake",
        ],
        outputs=["angle", "throttle", "brake"],
    )

    # アクチュエータの追加
    act = SerialTransmitter()
    car.add(act, inputs=["angle", "throttle", "brake"])

    # `tub`にデータを保存
    inputs = [
        "cam/image_array",
        "user/angle",
        "user/throttle",
        "user/brake",
        "user/mode",
    ]
    types = ["image_array", "float", "float", "float", "str"]

    # `tub`保存用のパーツを追加
    # 新しいレコードとして格納するのか、それとも既存のレコードに追加するのか
    tub_path = (
        TubHandler(path=cfg.DATA_PATH).create_tub_path()
        if cfg.AUTO_CREATE_NEW_TUB
        else cfg.DATA_PATH
    )
    tub_writer = TubWriter(base_path=tub_path, inputs=inputs, types=types)
    car.add(
        tub_writer,
        inputs=inputs,
        outputs=["tub/num_records"],
        run_condition="recording",
    )

    # メインループを開始
    car.start(rate_hz=cfg.DRIVE_LOOP_HZ, max_loop_count=cfg.MAX_LOOPS)


#######################################################


#
# basic.py
# - ヘルパークラスを引用
#
class PilotCondition:
    """制御者を決定するヘルパークラス"""

    def run(self, mode):
        return mode != "user"


class DriveMode:
    """AIとユーザ制御間のディスパッチを行うヘルパークラス"""

    def __init__(self, cfg):
        self.cfg = cfg

    def run(
        self,
        mode,
        user_angle,
        user_throttle,
        user_brake,
        pilot_angle,
        pilot_throttle,
        pilot_brake,
    ):
        if mode == "user":
            return user_angle, user_throttle, user_brake
        elif mode == "local_angle":
            return pilot_angle if pilot_angle else 0.0, user_throttle, user_brake
        else:
            return (
                pilot_angle if pilot_angle else 0.0,
                pilot_throttle * self.cfg.AI_THROTTLE_MULT if pilot_throttle else 0.0,
                # TODO ブレーキ制限スケールの処理
                pilot_brake * self.cfg.AI_THROTTLE_MULT if pilot_brake else 0.0,
            )


#
# actuator.py
#
class SerialTransmitter(object):
    """
    PIUS用シリアルトランスミッタ

    `run()`メソッドに与えられた三つの`float`型引数を
    `struct.pack()`でバイナリに変換し、シリアルポートに転送します。

    **GPIOピン対応表**
    6(GND)      -> 6(GND)
    8(UART TX)  -> 10(UART RX)
    10(UART RX) -> 8(UART TX)
    """

    def __init__(self):
        # ! ポートレートは変更すると不具合が多発します。
        self.ser = serial.Serial("/dev/ttyTHS1", 115200, timeout=1)
        # シリアル通信安定化のための待機
        time.sleep(2)

        self.steering = 1500
        self.throttle = 1000
        self.brake = 1948
        self.state = 1

    def run(self, steering, throttle, brake):
        self.run_threaded(steering, throttle, brake)

    def run_threaded(self, steering, throttle, brake):
        """引数をバイナリに変換して送信する"""
        self.pwm_s(steering)
        self.pwm_t(throttle)
        self.pwm_b(brake)

        self.shift(throttle)

        print(f"{self.steering}, {self.throttle}, {self.brake}, {self.state}")
        packet = struct.pack(
            "4h", self.steering, self.throttle, self.brake, self.state
        )
        self.ser.write(packet)

    def pwm_s(self, steering):
        min_ = 1.139
        def_ = 1.5
        max_ = 1.835

        steering = min(1.0, max(steering, -1.0))
        print(f"s: {steering}")
        if steering > 0:
            val = steering * (max_ - def_) + def_
        elif steering == 0:
            val = def_
        elif steering < 0:
            val = (1+steering) * (def_ - min_) + min_

        self.steering = int(val * 1000)

    def pwm_t(self, throttle):
        min_ = 1
        max_ = 2

        throttle = min(1.0, max(throttle, -1.0))
        val = abs(throttle) * (max_ - min_) + min_
        self.throttle = int(val * 1000)

    def pwm_b(self, brake):
        min_ = 1.135
        max_ = 1.948

        brake = min(1.0, max(brake, 0))
        val = brake * (max_ - min_) + min_
        self.brake = int(val * 1000)

    def shift(self, throttle):
        if throttle > 0:
            self.state = 0
        elif throttle == 0:
            self.state = 1
        elif throttle < 0:
            self.state = 2

    def shutdown(self):
        self.run_threaded(1500, 1000, 1948)
        self.ser.close()


#######################################################


if __name__ == "__main__":
    args = docopt(__doc__)
    cfg = dk.load_config()
    if args["drive"]:
        drive(cfg, model_path=args["--model"])
    elif args["train"]:
        train(
            cfg,
            tub_paths=args["--tubs"],
            model=args["--model"],
            transfer=args["--transfer"],
            comment=args["--comment"],
        )
