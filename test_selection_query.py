import requests
import json

# Test the selection query endpoint
url = "http://127.0.0.1:8000/api/v1/query/selection/"
headers = {
    "Content-Type": "application/json"
}
data = {
    "selected_text": "Physical AI Robotics is an emerging field that combines artificial intelligence with physical systems.",
    "question": "What does this text say about Physical AI Robotics?"
}

try:
    response = requests.post(url, headers=headers, data=json.dumps(data))
    print(f"Status Code: {response.status_code}")
    print(f"Response: {response.json()}")
except Exception as e:
    print(f"Error: {str(e)}")