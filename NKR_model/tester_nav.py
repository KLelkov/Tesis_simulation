
import csv
import rospy
from rdk_msgs.msg import locomotion
from rdk_msgs.msg import motors
from rdk_msgs.msg import navigation
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TwistWithCovarianceStamped


def main():
    rospy.init_node('tester_nav', anonymous=True)
    locomotionPub = rospy.Publisher('locomotion_data', locomotion, queue_size=1)
    motorsPub = rospy.Publisher('motors_data', motors, queue_size=1)
    gpsPosPub = rospy.Publisher('gps_node/fix', NavSatFix, queue_size=1)
    gpsVelPub = rospy.Publisher('gps_node/fix_velocity', TwistWithCovarianceStamped, queue_size=1)
    rate = rospy.Rate(100) # 100hz
    locomotionMsg = locomotion()
    motorsMsg = motors()
    posMsg = NavSatFix()
    velMsg = TwistWithCovarianceStamped()
    publishCounter = 0

    csvfile = open('sensors.csv', newline='')
    reader = csv.reader(csvfile, delimiter=',')
    time = []
    gammaf = []
    gammar = []
    gyro = []
    lat = []
    lon = []
    vn = []
    ve = []
    w1 = []
    w2 = []
    w3 = []
    w4 = []
    header = True
    for row in reader:
        if header:
            header = False
        else:
            time.append(long(row[0]))
            gammaf.append(float(row[1]))
            gammar.append(float(row[2]))
            gyro.append(float(row[3]))
            lat.append(float(row[4]))
            lon.append(float(row[5]))
            vn.append(float(row[6]))
            ve.append(float(row[7]))
            w1.append(float(row[8]))
            w2.append(float(row[9]))
            w3.append(float(row[10]))
            w4.append(float(row[11]))
    num = len(time)
    for i in range(0, num):
        motorsMsg.odo = [w1[i], w2[i], w3[i], w4[i]]
        motorsMsg.angleFront = gammaf[i]
        motorsMsg.angleRear = gammar[i]
        motorsPub.publish(motorsMsg)
        locomotionMsg.timestamp = time[i]
        locomotionMsg.angular_velocity = [0, 0, gyro[i]]
        locomotionPub.publish(locomotionMsg)
        if publishCounter % 100 == 0:
            velMsg.twist.twist.linear.y = vn[i];
            velMsg.twist.twist.linear.x = ve[i];
            gpsVelPub.publish(velMsg)
            posMsg.latitude = lat[i]
            posMsg.longitude = lon[i]
            posMsg.status.status = 1
            gpsPosPub.publish(posMsg)
        publishCounter += 1
        rate.sleep() # sleep to simulate 100 Hz update rate


if __name__ = "__main__":
    main()
