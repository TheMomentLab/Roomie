# models.py (명세서 기반으로 재구성)

from pydantic import BaseModel
from typing import List, Optional, Union

# --- 개별 요청 페이로드 모델 ---
# 각 action에 맞는 payload를 명확히 정의합니다.

class CreateCallTaskPayload(BaseModel):
    location_name: str
    task_type_id: int

class GetHistoryPayload(BaseModel):
    location_name: Optional[str] = None # get_call_history 용
    request_location: Optional[str] = None # get_order_history, get_task_list 용
    task_name: Optional[str] = None
    task_type_name: Optional[str] = None

class GetMenuPayload(BaseModel):
    location_name: str

class OrderItem(BaseModel):
    name: str
    quantity: int
    price: int

class OrderDetails(BaseModel):
    items: List[OrderItem]

class CreateDeliveryTaskPayload(BaseModel):
    location_name: str
    task_type_name: str
    order_details: OrderDetails

# --- 최상위 요청 모델 ---
# Union을 사용하여 action에 따라 올바른 payload 모델을 선택하도록 합니다.
class RequestModel(BaseModel):
    type: str
    action: str
    payload: Union[
        CreateCallTaskPayload,
        GetHistoryPayload,
        GetMenuPayload,
        CreateDeliveryTaskPayload
    ]