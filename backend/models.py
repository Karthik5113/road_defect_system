from sqlalchemy import Column, Integer, String, Float
from .database import Base

class Defect(Base):
    __tablename__ = "defects"

    id = Column(Integer, primary_key=True, index=True)
    defect_type = Column(String)
    latitude = Column(Float)
    longitude = Column(Float)
    confidence = Column(Float)
    date = Column(String)
    time = Column(String)