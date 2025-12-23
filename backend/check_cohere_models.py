import cohere
from src.config import settings

# Initialize Cohere client
client = cohere.Client(settings.COHERE_API_KEY)

# List available models
try:
    models = client.models.list()
    print("Available models:")
    for model in models:
        print(f"- {model.name}")
except Exception as e:
    print(f"Error listing models: {str(e)}")