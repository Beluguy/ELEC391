# Import the InferencePipeline object
from inference import InferencePipeline
from inference.core.interfaces.stream.sinks import render_boxes #Import pre-defined box visualization function
import time
import threading
import requests

#Global Variables
latest_inference = None
http_url = "http://172.20.10.10:81/stream" #Computer acts as client to recieve stream
rtsp_url = "rtsp://localhost:8554/stream" #Computer acts as server to delivery RTSP streams to inference pipeline client
esp32_ip = "172.20.10.10"

#@Function: Defines custom actions to be taken after each prediction
def custom_inference_sink(predictions, video_frame):
    global latest_inference

    #Capture latest prediction
    if predictions and 'predictions' in predictions and predictions['predictions']:
            latest_inference = predictions['predictions'][0]['class']
            print(latest_inference)

    #Output video with inference and bounding box
    render_boxes(predictions, video_frame)

#@Function: Call to pipeline to start in a seperate thread
def start_pipeline():
        pipeline.start()
        pipeline.join()

#@Function: Sends HTTP GET requests - delivering messages from client to server
def send_message(ip_address, message):
    url = f"http://{ip_address}/message?msg={message}"
    try:
        response = requests.get(url)
        if response.status_code == 200:
            print(f"Message '{message}' sent successfully")
        else:
            print(f"Failed to send message. Status code: {response.status_code}")
    except requests.RequestException as e:
        print(f"Error sending message: {e}")

#---------------------------------------------------------------------INFERENCE-----------------------------------------------------------------------------------
# initialize a pipeline object
pipeline = InferencePipeline.init(
    model_id="rock-paper-scissors-sxsw/11", # Roboflow model to use
    video_reference=rtsp_url, # Path to video, device id (int, usually 0 for built in webcams), or RTSP stream url
    on_prediction=custom_inference_sink, # Function to run after each prediction
)

pipeline_thread = threading.Thread(target=start_pipeline)
pipeline_thread.start()

while(True):
    send_message(esp32_ip, latest_inference)
    time.sleep(1)
