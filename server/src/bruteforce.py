# see https://blog.alexandruioan.me/2017/01/31/the-2017-university-of-bristol-arm-hackathon for more details
import math
from http.server import BaseHTTPRequestHandler, HTTPServer, urllib
from sys import argv
# import matplotlib.pyplot as plt
import serial
import threading
import queue
import numpy as np
import time

q = queue.Queue()

radius = 250

UP = b'c60'
DOWN = b'c120'

INIT_A = b'a180'
INIT_B = b'b180'

DET = b'd'
ATA = b'e'

# serial flag for debugging
SEND = True

def park_and_detach():
    if SEND:
        ser = serial.Serial('/dev/ttyACM0', 115200)
        ser.write(INIT_A)
        ser.write(INIT_B)
        ser.write(UP)
        ser.write(DET)

park_and_detach()

# separate thread for sending data to serial
def serial_send(q):
    while True:
        to_send = q.get()

        if SEND:
            ser.write(to_send)
            ser.flush()
        q.task_done()

t = threading.Thread(target=serial_send, args=(q,))
t.start()

class req_handler(BaseHTTPRequestHandler):

    prevX = -1
    prevY = -1

    # this runs when points are POSTed to the server
    # it triggers the calculation of the angles of the servos
    # and puts them in a queue
    # the tread at the other end of the queue sends the data over serial
    def do_POST(self):
        length = int(self.headers['Content-Length'])
        post_data = urllib.parse.parse_qs(self.rfile.read(length).decode('utf-8'))

        self.send_response(200)
        self.send_header('Content-type', 'text/html')
        self.end_headers()

        count = int((post_data['count'])[0])
        q.put_nowait(UP)

        # to_plot = []
        q.put_nowait(ATA)

        for i in range(count):
            pointX = float((post_data['p' + str(i) + 'x'])[0])
            pointY = float((post_data['p' + str(i) + 'y'])[0])

            # don't draw points that are too close
            # if (req_handler.prevX, req_handler.prevY) != (-1, -1):
            #     # print(math.sqrt((req_handler.prevX - pointX)**2 + (req_handler.prevY - pointY)**2))
            #     if math.sqrt((req_handler.prevX - pointX)**2 + (req_handler.prevY - pointY)**2) < 2:
            #         continue

            # timing
            # t0 = time.time()
            (theta1, theta2) = bruteforce(pointX, pointY)
            # t1 = time.time()
            # total = t1 - t0
            # print(total)

            lift = False

            # lift the pen if it has to move more than __ pixels,
            # in which case we probably don't want a continuous line
            if (req_handler.prevX, req_handler.prevY) != (-1, -1):
                if math.sqrt((req_handler.prevX - pointX)**2 + (req_handler.prevY - pointY)**2) > 30:
                    lift = True

            q.put_nowait(UP if lift else DOWN)

            # if the motors had a 360 degree range of motion you could get 2 solutions
            # (t11, t12), (t21, t22) = bruteforce360(pointX, pointY)
            # tx1 - main circle angle
            # tx2 - second circle angle

            # logic in case there are 2 solutions
            # theta1r = int(round(theta1))
            # theta2r = int(round(theta2))

            # t21r = int(round(t22))
            # t22r = int(round(t21))

            # print(t11r, t12r)
            # print(t21r, t22r)

            # if (t11r, t12r) == (0, 0):
            #     if (t21r, t22r) == (0, 0):
            #         (sol1, sol2) = (0, 0)
            #     else:
            #         (sol1, sol2) = (t21r, t22r)
            # else:
            #     if (t21r, t22r) == (0, 0):
            #         (sol1, sol2) = (t11r, t12r)
            #     else:
            #         # TODO: decision logic (find solution closest to the previous one?)
            #         (sol1, sol2) = (t11r, t12r)

            (sol1, sol2) = (int(round(theta1)), int(round(theta2)))

            a = str(sol1)
            b = str(sol2)

            to_send = ('a' + a + 'b' + b + '\n').encode('ascii')

            q.put_nowait(to_send)

            # save the current position
            (req_handler.prevX, req_handler.prevY) = (pointX, pointY)

            # save the points for plotting
            # to_plot.append((sol1, sol2, pointX, pointY))

            # plot point that needs to be drawn
            # plt.plot(pointX, pointY, 'ro')

        # draw the circles corresponding to the solution - this will take a long time
        # for theta in range(0, 360):
        #     for (t11, t21, pointX, pointY) in to_plot:
        #         center2X = radius * math.cos(math.radians(t21)) + radius * math.cos(math.radians(theta))
        #         center2Y = radius * math.sin(math.radians(t21)) + radius * math.sin(math.radians(theta))

        #         plt.plot(center2X, center2Y, 'rx')

                # center2X = radius * math.cos(math.radians(t21)) + radius * math.cos(math.radians(theta))
                # center2Y = radius * math.sin(math.radians(t21)) + radius * math.sin(math.radians(theta))

                # plt.plot(center2X, center2Y, 'rx')

        park_and_detach()

# start the HTTP server, binding it to all addresses, port 1180
def run(server_class = HTTPServer, handler_class = req_handler, port = 1180):
    server_address = ('0.0.0.0', port)
    httpd = server_class(server_address, handler_class)
    print('Starting httpd...')
    httpd.serve_forever()

def main():
    if len(argv) == 2:
        run(port = int(argv[1]))
    else:
        run()

# https://uk.mathworks.com/help/fuzzy/examples/modeling-inverse-kinematics-in-a-robotic-arm.html
# pregenerate grid
theta1range = np.arange(0, math.pi, 0.01)
theta2range = np.arange(0, math.pi, 0.01)

THETA1, THETA2 = np.meshgrid(theta1range, theta2range)

X_pred = radius * np.cos(THETA1) + radius * np.cos(THETA1 + THETA2)
Y_pred = radius * np.sin(THETA1) + radius * np.sin(THETA1 + THETA2)

def bruteforce(pointX, pointY):
    last_theta = -100
    switched = False

    list1 = []
    list2 = []

    min_dist = 100000
    min_theta1 = 0
    min_theta2 = 0

    # slow solution
    # ~0.12s
    # for theta1 in np.arange(0, math.pi, 0.01):
    #     for theta2 in np.arange(0, math.pi, 0.01):
    #         x_pred = radius * math.cos(theta1) + radius * math.cos(theta1 + theta2)
    #         y_pred = radius * math.sin(theta1) + radius * math.sin(theta1 + theta2)

    #         look_dist = math.sqrt((x_pred - pointX) ** 2 + (y_pred - pointY) ** 2)
    #         if look_dist < min_dist:
    #             min_dist = look_dist
    #             min_theta1 = theta1
    #             min_theta2 = theta2

    # numpy solution
    # ~0.005s
    # generate 3D array of repeated target point
    point = np.array([[[pointX, pointY]]])
    point3D = np.repeat(np.repeat(point, X_pred.shape[0], axis = 0), X_pred.shape[1], axis = 1)
    # create 3D array with potential X and Y values
    grid = np.stack((X_pred, Y_pred), axis = 2)
    # compute the Euclidean distance
    diff = np.subtract(point3D, grid)
    dists = np.linalg.norm(diff, ord = 2, axis = 2)
    # find the minimum distance (grid point closest to the target point)
    idx1, idx2 = np.unravel_index(dists.argmin(), dists.shape)
    # extract its theta values
    min_theta1 = THETA1[idx1][idx2]
    min_theta2 = THETA2[idx1][idx2]

    return (math.degrees(min_theta1), math.degrees(min_theta2))

# algorithm:
# sweep the main circle angle. each point on the circumference
# is the center of the second circle - a potential solution
# calculate the circle equation for the second one, and see if the
# target point satisfies it (with a tolerance)
# as we're dealing with pixels and not true geometrical points,
# it will clusters of solutions close together - average the points
# within the clusters
# there are up to two solutions - save these separately
def bruteforce360(pointX, pointY):
    last_theta = -100
    switched = False

    list1 = []
    list2 = []

    min_dist = 100000
    min_theta1 = 0
    min_theta2 = 0

    # theta - main circle angle
    for theta in range(0, 360):
        center2X = radius * math.cos(math.radians(theta))
        center2Y = radius * math.sin(math.radians(theta))

        # plt.plot(center2X, center2Y, 'bx')

        sr = ((center2X - pointX) ** 2) + ((center2Y - pointY) ** 2)

        if sr > (radius * 0.95)**2 and sr < (radius * 1.05) ** 2:
            # found the second solution, switch arrays
            if abs(last_theta - theta) > 30 and last_theta >= 0:
                switched = True

            # the angle of the 2nd circle is the angle between its center and the target point
            if switched:
                list1.append((theta, math.degrees(math.atan2(center2Y - pointY, center2X - pointX))))
            else:
                list2.append((theta, math.degrees(math.atan2(center2Y - pointY, center2X - pointX))))

            last_theta = theta

    sumt1 = 0
    sumt2 = 0

    # averaging

    # tx1 - main circle angle
    # tx2 - second circle angle
    for (t1, t2) in list1:
        sumt1 += t1
        sumt2 += t2

    t11 = sumt1 / len(list1) if len(list1) != 0 else 0
    t12 = sumt2 / len(list1) if len(list1) != 0 else 0

    sumt1 = 0
    sumt2 = 0

    # tx1 - main circle angle
    # tx2 - second circle angle
    for (t1, t2) in list2:
        sumt1 += t1
        sumt2 += t2

    t21 = sumt1 / len(list2) if len(list2) != 0 else 0
    t22 = sumt2 / len(list2) if len(list2) != 0 else 0

    return (t11, t12), (t21, t22)

if __name__ == "__main__":
    main()
