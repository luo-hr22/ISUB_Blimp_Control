import cv2  
from gevent import pywsgi   
from flask import Flask, Response  
import time  

app = Flask(__name__)  

# 摄像头设置  
camera = cv2.VideoCapture(0)  # 选择默认摄像头  

# 设置较低的分辨率  
camera.set(cv2.CAP_PROP_FRAME_WIDTH, 2560)  
camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)  

# 设定想要的帧率  
fps = 8  
frame_time = 1 / fps  

def generate_frame():  
    while True:  
        start_time = time.time()  
        success, frame = camera.read()  
        
        if not success:  
            print("Failed to capture image")  
            break  

        # 减少编码延迟，设置JPEG编码质量  
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 80]  # 80是压缩质量  
        ret, buffer = cv2.imencode('.jpg', frame, encode_param)  
        frame = buffer.tobytes()  

        # 控制帧率  
        elapsed_time = time.time() - start_time  
        if elapsed_time < frame_time:  
            time.sleep(frame_time - elapsed_time)  
        
        # 生成图像流  
        yield (b'--frame\r\n'  
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')  

@app.route('/video_feed1')  
def video_feed():  
    return Response(generate_frame(),  
                    mimetype='multipart/x-mixed-replace; boundary=frame')  

if __name__ == '__main__':  
    try:  
        # 使用gevent的WSGIServer来运行Flask应用  
        server = pywsgi.WSGIServer(('0.0.0.0', 8080), app)  
        server.serve_forever()  
    finally:  
        camera.release()  # 确保在结束时释放摄像头
