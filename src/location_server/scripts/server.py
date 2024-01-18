#!/usr/bin/env python3
import rospy
from apriltags2_ros.msg import AprilTagDetectionArray
import json
from http.server import BaseHTTPRequestHandler, HTTPServer
import threading

data = {
    'x': "",
    'y': ""
}

class TagInfoServer(BaseHTTPRequestHandler):
    def do_GET(self):
        self.send_response(200)
        self.send_header('Content-type', 'application/json')
        self.end_headers()
        self.wfile.write(json.dumps(data).encode('utf-8'))

def tag_callback(msg):
    global data
    if len(msg.detections) > 0:
        for tag in msg.detections:
            x = tag.pose.pose.pose.position.x
            y = tag.pose.pose.pose.position.y
            data['x'] = x
            data['y'] = y
            rospy.loginfo(data)

def run_server():
    server_address = ('', 8080)
    httpd = HTTPServer(server_address, TagInfoServer)
    rospy.loginfo('HTTP Server running on port 8080')
    httpd.serve_forever()

def main():
    rospy.init_node('tag_subscriber', anonymous=True)
    rospy.Subscriber('tag_detections', AprilTagDetectionArray, tag_callback)

    server_thread = threading.Thread(target=run_server)
    server_thread.daemon = True
    server_thread.start()

    rospy.spin()

if __name__ == '__main__':
    main()
