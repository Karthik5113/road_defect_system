from pydantic import BaseModel

class DefectCreate(BaseModel):
    defect_type: str
    latitude: float
    longitude: float
    confidence: float
    date: str
    time: str