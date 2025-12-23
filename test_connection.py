import requests
import json

# Test the backend connection with the correct endpoint
url = "http://127.0.0.1:8000/api/v1/query/"
headers = {
    "Content-Type": "application/json"
}
data = {
    "question": "What is Physical AI Robotics?",
    "session_id": "test-session-123"
}

try:
    response = requests.post(url, headers=headers, data=json.dumps(data))
    print(f"Status Code: {response.status_code}")
    print(f"Response: {response.json()}")
except Exception as e:
    print(f"Error: {str(e)}")