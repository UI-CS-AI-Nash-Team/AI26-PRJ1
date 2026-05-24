import heapq
import itertools

def ucs(initial_state):
    # الگوریتم UCS واسه پیدا کردن مسیر بهینه بر اساس هزینه خانه‌ها
    
    # این کانتر واسه اینه که اگه هزینه دوتا مسیر یکی شد، پایتون ارور مقایسه آبجکت نده
    counter = itertools.count()
    
    # صف اولویت (frontier): (هزینه کل، شماره گام، وضعیت فعلی، مسیر طی شده)
    frontier = []
    heapq.heappush(frontier, (0, next(counter), initial_state, []))
    
    # مجموعه گره‌های دیده شده واسه اینکه توی لوپ نیفتیم
    explored = set()
    
    while frontier:
        # اونی که هزینه‌ش از همه کمتره رو پاپ می‌کنیم
        current_cost, _, current_state, path = heapq.heappop(frontier)
        
        # اگه همه شیشه‌ها جمع شده بودن و رسیدیم به هدف، مسیر رو برگردون
        if current_state.is_goal_state():
            return path
            
        # اگه این وضعیت رو قبلاً با هزینه کمتر بررسی نکردیم، پردازشش کن
        if current_state not in explored:
            explored.add(current_state)
            
            # رفتن سراغ همسایه‌ها و حرکت‌های مجاز بعدی
            for action, step_cost, next_state in current_state.get_successors():
                
                # اگه حرکت بعدی مساوی بود با خوردن به نایت کینگ، این مسیر کلاً کنکله
                if next_state.is_collision_state():
                    continue
                
                # اگه جاده جدید بود، هزینه رو آپدیت کن و بذار توی صف
                if next_state not in explored:
                    new_cost = current_cost + step_cost
                    new_path = path + [action]
                    
                    heapq.heappush(frontier, (new_cost, next(counter), next_state, new_path))
                    
    return None # اگه هیچ مسیری پیدا نشد
