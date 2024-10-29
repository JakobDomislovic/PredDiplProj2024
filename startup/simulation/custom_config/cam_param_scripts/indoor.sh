sleep 5

BRIGHTNESS=0
CONTRAST=100
EXPOSURE=300
GAIN=50

echo "Setting: "
echo "  brightness: $BRIGHTNESS"
echo "  contrast: $CONTRAST"
echo "  exposure: $EXPOSURE"
echo "  gain: $GAIN"

rosservice call /camera/rgb_camera/set_parameters "config:
 bools:
 - {name: 'enable_auto_exposure', value: False}
 - {name: 'enable_auto_white_balance', value: False}
 - {name: 'auto_exposure_priority', value: False}
 ints:
 - {name: 'brightness', value: $BRIGHTNESS}
 - {name: 'contrast', value: $CONTRAST}
 - {name: 'exposure', value: $EXPOSURE}
 - {name: 'gain', value: $GAIN}"

