#!/usr/bin/env python3

# import necessary argumnets 
import gi
import cv2
import argparse
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# import required library like Gstreamer and GstreamerRtspServer
gi.require_version('Gst', '1.0')
gi.require_version('GstRtspServer', '1.0')
from gi.repository import Gst, GstRtspServer, GLib

fps = 15
image_width = 480
image_height = 270
port = 8554
stream_uri = "mystream"
frame = None

# Sensor Factory class which inherits the GstRtspServer base class and add
# properties to it.
class SensorFactory(GstRtspServer.RTSPMediaFactory):
    def __init__(self, **properties):
        super(SensorFactory, self).__init__(**properties)
        self.number_frames = 0
        self.fps = fps
        self.duration = 1 / self.fps * Gst.SECOND  # duration of a frame in nanoseconds
        self.launch_string = 'appsrc name=source is-live=true block=true format=GST_FORMAT_TIME ' \
                             'caps=video/x-raw,format=BGR,width={},height={},framerate={}/1 ' \
                             '! videoconvert ! video/x-raw,format=I420 ' \
                             '! x264enc speed-preset=ultrafast tune=zerolatency ' \
                             '! rtph264pay config-interval=1 name=pay0 pt=96' \
                             .format(image_width, image_height, self.fps)
                             
    # method to capture the video feed from the camera and push it to the
    # streaming buffer.
    def on_need_data(self, src, length):
        print ("need data")
        if frame is not None:
            # It is better to change the resolution of the camera 
            # instead of changing the image shape as it affects the image quality.
            frame = cv2.resize(frame, (image_width, image_height), \
                interpolation = cv2.INTER_LINEAR)
            data = frame.tostring()
            buf = Gst.Buffer.new_allocate(None, len(data), None)
            buf.fill(0, data)
            buf.duration = self.duration
            timestamp = self.number_frames * self.duration
            buf.pts = buf.dts = int(timestamp)
            buf.offset = timestamp
            self.number_frames += 1
            retval = src.emit('push-buffer', buf)
            print('pushed buffer, frame {}, duration {} ns, durations {} s'.format(self.number_frames,
                                                                                    self.duration,
                                                                                    self.duration / Gst.SECOND))

    # attach the launch string to the override method
    def do_create_element(self, url):
        return Gst.parse_launch(self.launch_string)
    
    # attaching the source element to the rtsp media
    def do_configure(self, rtsp_media):
        self.number_frames = 0
        appsrc = rtsp_media.get_element().get_child_by_name('source')
        appsrc.connect('need-data', self.on_need_data)

# Rtsp server implementation where we attach the factory sensor with the stream uri
class GstServer(GstRtspServer.RTSPServer):
    def __init__(self, **properties):
        super(GstServer, self).__init__(**properties)
        self.factory = SensorFactory()
        self.factory.set_shared(True)
        self.set_service(str(port))
        self.get_mount_points().add_factory(stream_uri, self.factory)
        self.attach(None)

def recieve_frame(img):
    cam_img = cv2.resize (bridge.imgmsg_to_cv2(img), (image_width, image_height))
    global frame
    frame = cam_img

def start_node():
    rospy.Subscriber('/front/color/image_raw', Image, recieve_frame)

    global bridge
    bridge = CvBridge()

    # initializing the threads and running the stream on loop.
    Gst.init(None)
    global server
    server = GstServer()
    loop = GLib.MainLoop()
    loop.run()

    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('video_stream_node', anonymous=True)
    rospy.loginfo("data stream node started")

    try:
        start_node()
    except rospy.ROSInterruptException:
        pass