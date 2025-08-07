// order.js
import { showToast } from './common.js';

export async function loadAndRenderMenu(orderType, containerId) {
    const isFood = orderType === 'food';
    const apiUrl = isFood ? `${window.API_URL}/get_food_menu` : `${window.API_URL}/get_supply_menu`;
    const action = isFood ? 'get_food_menu' : 'get_supply_menu';
    const container = document.getElementById(containerId);

    if (!container) return;
    container.innerHTML = '<p>메뉴를 불러오는 중입니다...</p>';

    try {
        const request = { type: "request", action, payload: { location_name: window.ROOM_ID } };
        const response = await fetch(apiUrl, { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify(request) });
        const result = await response.json();
        const items = isFood ? result.payload.food_items : result.payload.supply_items;

        if (items && items.length > 0) {
            renderMenu(items, orderType, container);
            updateTotalPrice(orderType);
        } else {
            container.innerHTML = '<p>메뉴 정보를 불러오는 데 실패했습니다.</p>';
        }
    } catch (error) {
        console.error(`${orderType} 메뉴 로딩 실패:`, error);
        container.innerHTML = '<p>오류가 발생했습니다. 잠시 후 다시 시도해주세요.</p>';
    }
}
function renderMenu(items, orderType, container) {
    container.innerHTML = '';
    const isFood = orderType === 'food';
    items.forEach((item, index) => {
        const menuItemElement = document.createElement('div');
        menuItemElement.className = 'menu-item';
        
        menuItemElement.innerHTML = `
            <div class="menu-image" style="background-image: url('${item.image || 'assets/images/default.png'}');"></div>
            <div class="menu-details">
                <div class="menu-info-top">
                    <span class="menu-name">${item.food_name || item.supply_name}</span>
                    ${isFood ? `<span class="menu-price">${item.price.toLocaleString()}원</span>` : ''}
                </div>
                <div class="quantity-control">
                    <button class="quantity-btn minus-btn" disabled></button>
                    <span class="quantity">0</span>
                    <button class="quantity-btn plus-btn"></button>
                </div>
                <button class="select-button" data-index="${index}" disabled>선택</button>
            </div>`;
        container.appendChild(menuItemElement);
    });
    addMenuEventListeners(orderType, items);
}

// ✅ 0-10 수량 제한 및 버튼 활성화/비활성화 로직 추가
function addMenuEventListeners(orderType, items) {
    document.querySelectorAll('.menu-item').forEach((menuItem) => {
        const index = parseInt(menuItem.querySelector('.select-button').dataset.index);
        const item = items[index];
        const qtySpan = menuItem.querySelector('.quantity');
        const minusBtn = menuItem.querySelector('.minus-btn');
        const plusBtn = menuItem.querySelector('.plus-btn');
        const selectBtn = menuItem.querySelector('.select-button');

        const updateButtons = (qty) => {
            minusBtn.disabled = qty <= 0;
            plusBtn.disabled = qty >= 10;
            selectBtn.disabled = qty <= 0;
        };
        
        plusBtn.addEventListener('click', () => {
            let qty = parseInt(qtySpan.textContent);
            if (qty < 10) {
                qty++;
                qtySpan.textContent = qty;
                updateButtons(qty);
            }
        });
        minusBtn.addEventListener('click', () => {
            let qty = parseInt(qtySpan.textContent);
            if (qty > 0) {
                qty--;
                qtySpan.textContent = qty;
                updateButtons(qty);
            }
        });
        selectBtn.addEventListener('click', () => {
            const quantity = parseInt(qtySpan.textContent);
            if(quantity > 0) {
                addToCart(item, quantity, orderType);
                // 선택 후 수량을 0으로 초기화
                qtySpan.textContent = 0;
                updateButtons(0);
            }
        });
    });
}

function addToCart(item, quantity, orderType) {
    const isFood = orderType === 'food';
    const cartKey = isFood ? 'cartData' : 'supplyCartData';
    let cart = JSON.parse(localStorage.getItem(cartKey)) || [];
    const itemName = isFood ? item.food_name : item.supply_name;
    const existingItem = cart.find(cartItem => cartItem.name === itemName);

    if (existingItem) {
        existingItem.quantity += quantity;
    } else {
        cart.push({ name: itemName, quantity, price: item.price || 0, image: item.image });
    }
    localStorage.setItem(cartKey, JSON.stringify(cart));
    
    // --- ✅ 수정된 부분 ---
    updateCartButton(orderType);
    updateTotalPrice(orderType); // 장바구니에 아이템 추가 후, 총액을 다시 계산하여 업데이트
    // -------------------
}


// ✅ 메뉴 화면 하단 총액 업데이트 함수
function updateTotalPrice(orderType) {
    const isFood = orderType === 'food';
    if (!isFood) return; // 비품은 총액 표시 안함

    const cartKey = 'cartData';
    const cart = JSON.parse(localStorage.getItem(cartKey)) || [];
    const totalPrice = cart.reduce((sum, item) => sum + (item.price * item.quantity), 0);
    
    const totalPriceElement = document.getElementById('total_price_footer');
    if (totalPriceElement) {
        totalPriceElement.textContent = `총 ${totalPrice.toLocaleString()}원`;
    }
}

export function updateCartButton(orderType) {
    const cartKey = orderType === 'food' ? 'cartData' : 'supplyCartData';
    const cart = JSON.parse(localStorage.getItem(cartKey)) || [];
    const cartButton = document.getElementById('button_cart');
    if (cartButton) {
        const itemCount = cart.reduce((sum, item) => sum + item.quantity, 0);
        cartButton.textContent = itemCount > 0 ? `장바구니 보기 (${itemCount})` : '장바구니 보기';
    }
}

export function renderCartContents(containerId, orderType) {
    const container = document.getElementById(containerId);
    if (!container) return;
    const isFood = orderType === 'food';
    const cartKey = isFood ? 'cartData' : 'supplyCartData';
    let cart = JSON.parse(localStorage.getItem(cartKey)) || [];

    if (cart.length === 0) {
        container.innerHTML = '<p style="text-align:center; color:#888;">장바구니가 비어 있습니다.</p>';
        if (isFood) { document.getElementById('total-price-display').textContent = '0원'; }
        document.getElementById('btn-order').disabled = true;
        return;
    }

    container.innerHTML = '';
    let totalPrice = 0;
    cart.forEach((item, index) => {
        const itemElement = document.createElement('div');
        itemElement.className = 'item';
        itemElement.innerHTML = `
            <div class="item-image" style="background-image: url('${item.image}')"></div>
            <div class="item-details">
                <span class="item-name">${item.name}</span>
                <div class="quantity-control">
                    <button class="quantity-btn minus-btn" data-index="${index}"></button>
                    <span class="quantity">${item.quantity}</span>
                    <button class="quantity-btn plus-btn" data-index="${index}"></button>
                </div>
                ${isFood ? `<span class="item-price">${(item.price * item.quantity).toLocaleString()}원</span>` : ''}
            </div>
            <button class="delete-btn" data-index="${index}">삭제</button>`;
        container.appendChild(itemElement);
        if (isFood) { totalPrice += item.price * item.quantity; }
    });

    if (isFood) { document.getElementById('total-price-display').textContent = `${totalPrice.toLocaleString()}원`; }
    document.getElementById('btn-order').disabled = false;
    addCartEventListeners(containerId, orderType);
}

function addCartEventListeners(containerId, orderType){
    const cartKey = orderType === 'food' ? 'cartData' : 'supplyCartData';
    let cart = JSON.parse(localStorage.getItem(cartKey)) || [];
    const updateAndSaveCart = () => {
        localStorage.setItem(cartKey, JSON.stringify(cart));
        renderCartContents(containerId, orderType);
    };

    document.querySelectorAll('.item .plus-btn').forEach(btn => {
        btn.addEventListener('click', (e) => {
            const index = parseInt(e.currentTarget.dataset.index);
            cart[index].quantity = Math.min(10, cart[index].quantity + 1);
            updateAndSaveCart();
        });
    });
    document.querySelectorAll('.item .minus-btn').forEach(btn => {
        btn.addEventListener('click', (e) => {
            const index = parseInt(e.currentTarget.dataset.index);
            if (cart[index].quantity > 1) {
                cart[index].quantity -= 1;
                updateAndSaveCart();
            }
        });
    });
    document.querySelectorAll('.item .delete-btn').forEach(btn => {
        btn.addEventListener('click', (e) => {
            const index = parseInt(e.currentTarget.dataset.index);
            cart.splice(index, 1);
            updateAndSaveCart();
        });
    });
}

export async function createOrder(orderType) {
    const isFood = orderType === 'food';
    const cartKey = isFood ? 'cartData' : 'supplyCartData';
    const cart = JSON.parse(localStorage.getItem(cartKey)) || [];

    if (cart.length === 0) { return alert("장바구니가 비어 있습니다."); }

    const request = {
        type: "request", action: "create_delivery_task",
        payload: {
            location_name: window.ROOM_ID,
            task_type_name: isFood ? "음식배송" : "비품배송",
            order_details: { items: cart.map(item => ({ name: item.name, quantity: item.quantity, price: item.price })) }
        }
    };
    try {
        const response = await fetch(`${window.API_URL}/create_delivery_task`, { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify(request) });
        const result = await response.json();
        if (result.payload && result.payload.success) {
            const taskName = result.payload.task_name;
            const taskType = isFood ? "음식배송" : "비품배송";
            
            // LocalStorage 대신 URL로 정보를 전달하도록 변경
            localStorage.removeItem(cartKey);
            location.hash = `order-success&task=${taskName}&type=${taskType}`;
        } 
        else 
        {
            showToast("../assets/images/error_toast.png");
        }
    } catch (err) {
        console.error("주문 요청 실패:", err);
        showToast("../assets/images/error_toast.png");
    }
}