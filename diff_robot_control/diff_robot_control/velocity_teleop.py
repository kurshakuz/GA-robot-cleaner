import sys

import geometry_msgs.msg
import rclpy

if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty


msg = """
This node takes keypresses from the keyboard and publishes them
as Twist messages. It works best with a US keyboard layout.
---------------------------
Controlling wheels separately:
   e    t    i
   d    g    k
(left)(both)(right)
anything else : stop
CTRL-C to quit
"""

moveBindings = {
    'e': (1, 0),
    'd': (-1, 0),
    'i': (0, 1),
    'k': (0, -1),
    't': (1, 1),
    'g': (-1, -1),
    # 'x': (0, 0),
}

def getKey(settings):
    if sys.platform == 'win32':
        # getwch() returns a string on Windows
        key = msvcrt.getwch()
    else:
        tty.setraw(sys.stdin.fileno())
        # sys.stdin.read() returns a string on Linux
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def saveTerminalSettings():
    if sys.platform == 'win32':
        return None
    return termios.tcgetattr(sys.stdin)


def restoreTerminalSettings(old_settings):
    if sys.platform == 'win32':
        return
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


def vels(v_l, v_r):
    return 'currently:\tleft %s\tright %s ' % (v_l, v_r)


def main():
    settings = saveTerminalSettings()

    rclpy.init()

    node = rclpy.create_node('teleop_twist_keyboard')
    pub = node.create_publisher(geometry_msgs.msg.Twist, 'cmd_vel', 10)

    speed = 0.5
    turn = 1.0
    v_l = 0.0
    v_r = 0.0
    dtheta = 0.0
    wheelSeparation = 0.4
    wheelDiameter = 0.2

    try:
        print(msg)
        print(vels(v_l, v_r))
        while True:
            key = getKey(settings)
            if key in moveBindings.keys():
                v_l += moveBindings[key][0]
                v_r += moveBindings[key][1]
            else:
                v_l = 0.0
                v_r = 0.0
                if (key == '\x03'):
                    break
            if v_r == v_l:
                twist = geometry_msgs.msg.Twist()
                twist.linear.x = v_r * speed
                twist.linear.y = 0.0
                twist.linear.z = 0.0
                twist.angular.x = 0.0
                twist.angular.y = 0.0
                twist.angular.z = 0.0
                pub.publish(twist)
            else:
                sr = v_r*wheelDiameter/2
                sl = v_l*wheelDiameter/2
                dtheta = (sr - sl)/wheelSeparation
                # print(sr, sl, dtheta)

                twist = geometry_msgs.msg.Twist()
                twist.linear.x = (v_r+v_l)/2 * speed
                twist.linear.y = 0.0
                twist.linear.z = 0.0
                twist.angular.x = 0.0
                twist.angular.y = 0.0
                twist.angular.z = dtheta * turn
                pub.publish(twist)
            print(vels(v_l, v_r))

    except Exception as e:
        print(e)

    finally:
        twist = geometry_msgs.msg.Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        pub.publish(twist)

        restoreTerminalSettings(settings)


if __name__ == '__main__':
    main()