import asyncio
import asyncpg
from src.config import settings

async def setup_database():
    """Setup the database tables for the RAG chatbot"""
    
    # Connect to the database
    conn = None
    try:
        print("Connecting to database...")
        conn = await asyncpg.connect(settings.NEON_DATABASE_URL)
        
        print("Reading database setup script...")
        with open("setup_db.sql", "r", encoding="utf-8") as f:
            sql_script = f.read()
        
        print("Executing database setup script...")
        await conn.execute(sql_script)
        
        print("SUCCESS: Database setup completed successfully!")
        
    except Exception as e:
        print(f"ERROR: Error setting up database: {str(e)}")
    finally:
        if conn:
            await conn.close()

if __name__ == "__main__":
    asyncio.run(setup_database())