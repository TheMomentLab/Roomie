// js/food.js

/**
 * 음식 주문 모듈
 * - 메뉴 조회 및 장바구니 처리
 * - 주문 완료 시 selectedTask 저장 후 상태 페이지로 이동
 */

export async function loadFoodMenu() {
  const request = {
    type: "request",
    action: "get_food_menu",
    payload: { location_name: ROOM_ID }
  };

  try {
    const response = await fetch(`${API_URL}/get_food_menu`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(request)
    });

    const result = await response.json();
    if (result.type === "response" && result.payload.food_items) {
      localStorage.setItem("foodMenuData", JSON.stringify(result.payload.food_items));
      renderFoodMenu(result.payload.food_items);
    } else {
      showToast( "error");
    }
  } catch (error) {
    console.error("음식 메뉴 불러오기 실패:", error);
    showToast( "error");
  }
}

function renderFoodMenu(items) {
  const container = document.getElementById("food-menu-list");
  if (!container) return;

  container.innerHTML = items.map((item, index) => `
    <div class="card" data-id="${index}">
      <img src="${item.image}" />
      <p>${item.food_name}</p>
      <p>${item.price.toLocaleString()}원</p>
      <button class="btn-add-food">추가</button>
    </div>
  `).join("");

  container.querySelectorAll(".btn-add-food").forEach((btn, i) => {
    btn.addEventListener("click", () => {
      const item = items[i];
      addToCart(item);
      showToast("asset/confirm_toast.png", "success");
    });
  });
}

function addToCart(item) {
  let cart = JSON.parse(localStorage.getItem("cartData")) || [];
  const existing = cart.find(i => i.name === item.food_name);
  if (existing) {
    existing.quantity += 1;
  } else {
    cart.push({
      name: item.food_name,
      quantity: 1,
      price: item.price,
      image: item.image
    });
  }
  localStorage.setItem("cartData", JSON.stringify(cart));
}

export function renderCart(containerId, orderButtonId) {
  const container = document.getElementById(containerId);
  const orderBtn = document.getElementById(orderButtonId);
  let cart = JSON.parse(localStorage.getItem("cartData")) || [];

  function updateCart() {
    if (!container) return;
    container.innerHTML = "";

    if (cart.length === 0) {
      container.innerHTML = '<p style="text-align:center;color:#888;">장바구니가 비어 있습니다.</p>';
      if (orderBtn) orderBtn.disabled = true;
      return;
    }

    cart.forEach((item, index) => {
      const el = document.createElement("div");
      el.className = "item";
      el.innerHTML = `
        <div class="item-image" style="background-image: url('${item.image}')"></div>
        <div class="item-details">
          <span class="item-name">${item.name}</span>
          <div class="quantity-control">
            <button class="quantity-btn" data-action="-1">-</button>
            <span class="quantity">${item.quantity}</span>
            <button class="quantity-btn" data-action="1">+</button>
          </div>
          <span class="item-price">${(item.price * item.quantity).toLocaleString()}원</span>
        </div>
        <button class="delete-btn">삭제</button>
      `;

      el.querySelectorAll(".quantity-btn").forEach(btn => {
        btn.addEventListener("click", () => {
          const change = parseInt(btn.dataset.action);
          cart[index].quantity = Math.min(10, Math.max(1, cart[index].quantity + change));
          saveCart();
        });
      });

      el.querySelector(".delete-btn").addEventListener("click", () => {
        cart.splice(index, 1);
        saveCart();
      });

      container.appendChild(el);
    });

    if (orderBtn) orderBtn.disabled = false;
  }

  function saveCart() {
    localStorage.setItem("cartData", JSON.stringify(cart));
    updateCart();
  }

  updateCart();

  // ✅ 주문 버튼 클릭 시 최소 정보만 저장하고 이동
  if (orderBtn) {
    orderBtn.addEventListener("click", async () => {
      const request = {
        type: "request",
        action: "create_delivery_task",
        payload: {
          location_name: ROOM_ID,
          task_type_name: "음식배송",
          order_details: {
            items: cart.map(item => ({
              name: item.name,
              quantity: item.quantity,
              price: item.price
            }))
          }
        }
      };

      try {
        const response = await fetch(`${API_URL}/create_delivery_task`, {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify(request)
        });

        const result = await response.json();
        if (result.type === "response" && result.payload.success) {
          localStorage.setItem("selectedTask", result.payload.task_name);
          localStorage.setItem("selectedTaskType", "음식배송");
          localStorage.removeItem("cartData");
          window.location.href = `index.html#status-history&task=${result.payload.task_name}`;
        } else {
          showToast ("error");
        }
      } catch (err) {
        console.error("주문 요청 실패:", err);
        showToast("error");
      }
    });
  }
}
