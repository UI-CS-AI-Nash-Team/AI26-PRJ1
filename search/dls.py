def dls_core(initial_state, limit):
    # هسته اصلی DLS واسه سرچ تا یه عمق خاص (limit)
    
    # پشته (Stack): (وضعیت فعلی، مسیر طی شده، عمق فعلی)
    stack = [(initial_state, [], 0)]
    
    # اینجا کلیدها میشن اثر انگشت وضعیت (state_id) و مقدارها میشن کمترین عمقی که بهش رسیدیم
    explored = {}
    
    while stack:
        current_state, path, depth = stack.pop()
        
        # چک کردن هدف (اسم تابع دقیقاً طبق پی‌دی‌اف استاد تنظیم شده)
        if current_state.state_goal_is():
            return path
            
        # فقط اگه هنوز به سقف عمق (limit) نرسیدیم، گره رو باز می‌کنیم
        if depth < limit:
            
            # ساختن اثر انگشت وضعیت با توابع دقیق پروژه
            state_id = (
                current_state.position_agent_get(),
                tuple(sorted(current_state.positions_targets_get())),
                current_state.position_enemy_get()
            )
            
            if state_id not in explored or depth < explored[state_id]:
                explored[state_id] = depth
                
                # گرفتن حرکات مجاز
                for action, step_cost, next_state in current_state.successors_get():
                    
                    # برخورد با نایت کینگ = کنسلی مسیر
                    if next_state.state_collision_is():
                        continue
                        
                    # وضعیت جدید رو می‌ندازیم تو پشته
                    stack.append((next_state, path + [action], depth + 1))
                    
    return None


def dls(problem):
    """
    Depth Limited Search (DLS) -> به روش IDS برای پیدا کردن بهینه‌ترین مسیر
    """
    # گرفتن وضعیت اولیه از آبجکت مسئله
    initial_state = problem.get_initial_state()
    
    # یه سقف بالا می‌ذاریم که تو مپ‌های هارد هم جواب بده
    max_possible_depth = 1000 
    
    for limit in range(max_possible_depth):
        result = dls_core(initial_state, limit)
        
        # اگه جواب پیدا شد همونو برگردون، در غیر این صورت برو عمق بعدی
        if result is not None:
            return result
            
    return None
