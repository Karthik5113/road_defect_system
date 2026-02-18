from fastapi import FastAPI, Depends
from sqlalchemy.orm import Session
from database import SessionLocal, engine
import models, schemas

models.Base.metadata.create_all(bind=engine)

app = FastAPI()

def get_db():
    db = SessionLocal()
    try:
        yield db
    finally:
        db.close()

@app.get("/")
def home():
    return {"status": "API RUNNING"}

@app.post("/upload")
def upload(defect: schemas.DefectCreate, db: Session = Depends(get_db)):
    new_defect = models.Defect(**defect.dict())
    db.add(new_defect)
    db.commit()
    return {"message": "stored"}

@app.get("/defects")
def get_defects(db: Session = Depends(get_db)):
    return db.query(models.Defect).all()