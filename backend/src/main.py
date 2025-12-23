from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

from .config import settings
from .routers.ingest_router import ingest_router
from .routers.query_router import query_router


def create_app() -> FastAPI:
    app = FastAPI(
        title=settings.PROJECT_NAME,
        description="RAG Chatbot Backend API for AI-Spec-Driven Book",
        version="0.1.0",
        docs_url="/docs",
        redoc_url="/redoc",
        openapi_url="/openapi.json"
    )

    # ✅ FIXED CORS (no trailing slash)
    app.add_middleware(
        CORSMiddleware,
        allow_origins=["https://physical-ai-humanoid-robotics-theta.vercel.app/"],
        allow_credentials=True,
        allow_methods=["*"],
        allow_headers=["*"],
    )

    app.include_router(ingest_router, prefix=settings.API_V1_STR, tags=["ingest"])
    app.include_router(query_router, prefix=settings.API_V1_STR, tags=["query"])

    @app.get("/")
    def read_root():
        return {"message": "RAG Chatbot Backend API", "version": "0.1.0"}

    @app.get("/health")
    def health_check():
        return {"status": "healthy", "service": "RAG Chatbot Backend"}

    return app


app = create_app()


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(
        "src.main:app",
        host="0.0.0.0",
        port=8000,
        reload=True
    )
