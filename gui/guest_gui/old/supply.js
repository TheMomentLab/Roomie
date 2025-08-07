// js/supply.js

/**
 * 비품 주문 모듈
 * - 메뉴 조회, 장바구니 관리, 주문 처리
 * - 주문 성공 시 selectedTask 저장 후 상태 페이지 이동
 */

export async function loadSupplyMenu() {
  const request = {
    type: "request",
    action: "get_supply_menu",
    payload: { location_name: ROOM_ID }
  };

  try {
    const response = await fetch(`${API_URL}/get_supply_menu`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(request)
    });

    const result = await response.json();
    if (result.type === "response" && result.payload.supplies) {
      const items = result.payload.supplies;
      localStorage.setItem("supplyMenuData", JSON.stringify(items));
      renderSupplyMenu(items);
    } else {
      showToast("asset/error_toast.png", "error");
    }
  } catch (err) {
    console.error("비품 목록 불러오기 실패:", err);
    showToast("asset/error_toast.png", "error");
  }
}

function renderSupplyMenu(items) {
  const container = document.getElementById("supply-menu-list");
  if (!container) return;

  container.innerHTML = items.map((item, index) => `
    <div class="card" data-id="${index}">
      <img src="${item.image || 'asset/default.png'}" />
      <p>${item.name}</p>
      <div class="quantity-control">
        <button class="quantity-btn" data-action="-1">-</button>
        <span class="quantity">1</span>
        <button class="quantity-btn" data-action="1">+</button>
      </div>
      <button class="btn-add-supply">선택</button>
    </div>
  `).join("");

  container.querySelectorAll(".card").forEach((card, i) => {
    const item = items[i];
    let quantity = 1;

    const qtySpan = card.querySelector(".quantity");
    card.querySelectorAll(".quantity-btn").forEach(btn => {
      btn.addEventListener("click", () => {
        const diff = parseInt(btn.dataset.action);
        quantity = Math.min(10, Math.max(1, quantity + diff));
        qtySpan.textContent = quantity;
      });
    });

    card.querySelector(".btn-add-supply").addEventListener("click", () => {
      addToCart(item, quantity);
      showToast("asset/confirm_toast.png", "success");
    });
  });
}

function addToCart(item, quantity) {
  let cart = JSON.parse(localStorage.getItem("supplyCartData")) || [];
  const existing = cart.find(i => i.name === item.name);
  if (existing) {
    existing.quantity += quantity;
  } else {
    cart.push({ name: item.name, quantity, image: item.image });
  }
  localStorage.setItem("supplyCartData", JSON.stringify(cart));
}

export function renderCart(containerId, orderButtonId) {
  const container = document.getElementById(containerId);
  const orderBtn = document.getElementById(orderButtonId);
  let cart = JSON.parse(localStorage.getItem("supplyCartData")) || [];

  function updateCart() {
    if (!container) return;
    container.innerHTML = cart.length === 0
      ? '<p style="text-align:center;color:#888;">장바구니가 비어 있습니다.</p>'
      : "";

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

    if (orderBtn) orderBtn.disabled = cart.length === 0;
  }

  function saveCart() {
    localStorage.setItem("supplyCartData", JSON.stringify(cart));
    updateCart();
  }

  updateCart();

  // ✅ 주문 버튼 클릭 시 최소 작업 정보만 저장하고 상태 페이지 이동
  if (orderBtn) {
    orderBtn.addEventListener("click", async () => {
      const payload = {
        type: "request",
        action: "create_delivery_task",
        payload: {
          location_name: ROOM_ID,
          task_type_name: "비품배송",
          order_details: {
            items: cartData.map(item => ({
              name: item.name,
              quantity: item.quantity,
              price: item.price
            }))
          }
        }
      };

      const request = {
        type: "request",
        action: "create_supply_task",
        payload
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
          localStorage.setItem("selectedTaskType", "비품배송");
          localStorage.removeItem("supplyCartData");
          window.location.href = `index.html#status-history&task=${result.payload.task_name}`;
        } else {
          showToast("asset/error_toast.png", "error");
        }
      } catch (err) {
        console.error("비품 요청 실패:", err);
        showToast("asset/error_toast.png", "error");
      }
    });
  }
}
