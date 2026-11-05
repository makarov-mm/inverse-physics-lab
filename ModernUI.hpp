#pragma once
// Native Windows controls retain their keyboard, focus and accessibility behavior.
// Only their presentation is replaced. No external UI runtime is required.
#include <windows.h>
#include <windowsx.h>
#include <commctrl.h>
#include <dwmapi.h>
#include <uxtheme.h>
#include <algorithm>
#include <string>
#include <vector>

#pragma comment(lib, "uxtheme.lib")
#pragma comment(lib, "dwmapi.lib")
#pragma comment(                                                                                             \
    linker,                                                                                                  \
    "/manifestdependency:\"type='win32' name='Microsoft.Windows.Common-Controls' version='6.0.0.0' processorArchitecture='*' publicKeyToken='6595b64144ccf1df' language='*'\"")

namespace modern {
inline constexpr COLORREF background = RGB(246, 248, 252), ink = RGB(25, 37, 57), muted = RGB(85, 101, 123),
                          accent = RGB(42, 99, 228), border = RGB(216, 224, 235), white = RGB(255, 255, 255),
                          canvas = RGB(3, 5, 8);
struct Surface;
enum class Kind { label, button, check, combo, slider };
struct Control {
    Surface *surface{};
    HWND hwnd{};
    Kind kind{};
    RECT logical{};
    bool hover{}, title{}, subtitle{}, status{}, card{}, instructions{};
};
struct Surface {
    HWND root{}, panel{};
    int width{}, offset{}, content{}, page{};
    bool highContrast{};
    HFONT body{}, smallFont{}, heading{}, strong{};
    HBRUSH panelBrush{}, inputBrush{};
    std::vector<Control *> controls;
    ~Surface() {
        for (auto f : {body, smallFont, heading, strong})
            DeleteObject(f);
        DeleteObject(panelBrush);
        DeleteObject(inputBrush);
    }
};
inline std::wstring text(HWND hwnd) {
    std::wstring value(static_cast<size_t>(GetWindowTextLengthW(hwnd)) + 1, L'\0');
    value.resize(static_cast<size_t>(GetWindowTextW(hwnd, value.data(), static_cast<int>(value.size()))));
    return value;
}
inline void rounded(HDC dc, RECT r, COLORREF fill, COLORREF stroke, int radius = 8) {
    auto brush = CreateSolidBrush(fill);
    auto pen = CreatePen(PS_SOLID, 1, stroke);
    auto oldBrush = SelectObject(dc, brush), oldPen = SelectObject(dc, pen);
    RoundRect(dc, r.left, r.top, r.right, r.bottom, radius, radius);
    SelectObject(dc, oldBrush);
    SelectObject(dc, oldPen);
    DeleteObject(brush);
    DeleteObject(pen);
}
inline void line(HDC dc, int x1, int y1, int x2, int y2, COLORREF color, int width = 1) {
    auto pen = CreatePen(PS_SOLID, width, color);
    auto old = SelectObject(dc, pen);
    MoveToEx(dc, x1, y1, nullptr);
    LineTo(dc, x2, y2);
    SelectObject(dc, old);
    DeleteObject(pen);
}
inline void refreshContrast(Surface &s) {
    HIGHCONTRASTW contrast{sizeof(contrast)};
    SystemParametersInfoW(SPI_GETHIGHCONTRAST, sizeof(contrast), &contrast, 0);
    s.highContrast = (contrast.dwFlags & HCF_HIGHCONTRASTON) != 0;
}
inline void position(Surface &s) {
    RECT client{};
    GetClientRect(s.panel, &client);
    s.page = client.bottom;
    s.offset = std::clamp(s.offset, 0, std::max(0, s.content - s.page));
    SCROLLINFO info{sizeof(info), SIF_RANGE | SIF_PAGE | SIF_POS};
    info.nMax = std::max(0, s.content - 1);
    info.nPage = static_cast<UINT>(s.page);
    info.nPos = s.offset;
    SetScrollInfo(s.panel, SB_VERT, &info, TRUE);
    GetClientRect(s.panel, &client);
    for (auto *c : s.controls) {
        if (GetParent(c->hwnd) != s.panel)
            continue;
        const auto &r = c->logical;
        // Reserve the native scrollbar gutter without cropping the controls.
        const int right = MulDiv(r.right, client.right, s.width);
        const int left = MulDiv(r.left, client.right, s.width);
        SetWindowPos(c->hwnd, nullptr, left, r.top - s.offset, right - left,
                     c->kind == Kind::combo ? 240 : r.bottom - r.top, SWP_NOZORDER | SWP_NOACTIVATE);
    }
    InvalidateRect(s.panel, nullptr, FALSE);
}
inline void scroll(Surface &s, int pixels) {
    s.offset += pixels;
    position(s);
}
inline void reveal(Control &c) {
    auto &s = *c.surface;
    if (GetParent(c.hwnd) != s.panel)
        return;
    if (c.logical.top < s.offset)
        s.offset = c.logical.top;
    else if (c.logical.bottom > s.offset + s.page)
        s.offset = c.logical.bottom - s.page;
    position(s);
}
inline void paint(Control &c, HDC target) {
    auto &s = *c.surface;
    RECT r{};
    GetClientRect(c.hwnd, &r);
    if (r.right <= 0 || r.bottom <= 0)
        return;
    HDC dc = CreateCompatibleDC(target);
    auto bitmap = CreateCompatibleBitmap(target, r.right, r.bottom);
    auto oldBitmap = SelectObject(dc, bitmap);
    auto bg = CreateSolidBrush(c.status ? canvas : background);
    FillRect(dc, &r, bg);
    DeleteObject(bg);
    SetBkMode(dc, TRANSPARENT);
    auto font =
        c.title ? s.heading : (c.subtitle || c.instructions || c.status || c.card ? s.smallFont : s.body);
    if (c.kind == Kind::button)
        font = s.strong;
    auto oldFont = SelectObject(dc, font);
    bool enabled = IsWindowEnabled(c.hwnd) != FALSE;
    bool focus = GetFocus() == c.hwnd;
    auto foreground = enabled ? ink : muted;
    auto value = text(c.hwnd);
    RECT content = r;
    UINT format = DT_SINGLELINE | DT_VCENTER | DT_END_ELLIPSIS | DT_NOPREFIX;
    if (c.kind == Kind::button) {
        const bool primary = GetDlgCtrlID(c.hwnd) == 30;
        const bool pressed = (SendMessageW(c.hwnd, BM_GETSTATE, 0, 0) & BST_PUSHED) != 0;
        COLORREF fill = primary ? accent : white;
        if (c.hover)
            fill = primary ? RGB(32, 83, 201) : RGB(234, 241, 255);
        if (pressed)
            fill = primary ? RGB(24, 67, 170) : RGB(219, 231, 253);
        if (!enabled)
            fill = RGB(232, 237, 244);
        InflateRect(&content, -1, -1);
        rounded(dc, content, fill, focus ? accent : (primary ? fill : border));
        if (focus) {
            RECT ring = content;
            InflateRect(&ring, -3, -3);
            DrawFocusRect(dc, &ring);
        }
        foreground = primary && enabled ? white : foreground;
        format |= DT_CENTER;
    } else if (c.kind == Kind::check) {
        const bool checked = SendMessageW(c.hwnd, BM_GETCHECK, 0, 0) == BST_CHECKED;
        int y = (r.bottom - 18) / 2;
        RECT box{1, y, 19, y + 18};
        rounded(dc, box, checked ? accent : white, focus || c.hover || checked ? accent : border, 5);
        if (checked) {
            line(dc, 5, y + 9, 9, y + 13, white, 2);
            line(dc, 9, y + 13, 15, y + 5, white, 2);
        }
        content.left = 28;
        if (focus) {
            RECT ring = r;
            InflateRect(&ring, -1, -1);
            DrawFocusRect(dc, &ring);
        }
    } else if (c.kind == Kind::combo) {
        InflateRect(&content, -1, -1);
        rounded(dc, content, white, focus || c.hover ? accent : border);
        auto selection = SendMessageW(c.hwnd, CB_GETCURSEL, 0, 0);
        if (selection != CB_ERR) {
            auto length = SendMessageW(c.hwnd, CB_GETLBTEXTLEN, selection, 0);
            if (length >= 0) {
                value.resize(static_cast<size_t>(length) + 1);
                SendMessageW(c.hwnd, CB_GETLBTEXT, selection, reinterpret_cast<LPARAM>(value.data()));
                value.resize(static_cast<size_t>(length));
            }
        }
        int x = r.right - 18, y = r.bottom / 2;
        line(dc, x - 4, y - 2, x, y + 2, muted, 2);
        line(dc, x, y + 2, x + 4, y - 2, muted, 2);
        content.left += 10;
        content.right -= 32;
    } else if (c.kind == Kind::slider) {
        RECT thumb{}, channel{};
        SendMessageW(c.hwnd, TBM_GETTHUMBRECT, 0, reinterpret_cast<LPARAM>(&thumb));
        SendMessageW(c.hwnd, TBM_GETCHANNELRECT, 0, reinterpret_cast<LPARAM>(&channel));
        const int x = (thumb.left + thumb.right) / 2, y = (thumb.top + thumb.bottom) / 2;
        RECT rail{channel.left, y - 2, channel.right, y + 2};
        rounded(dc, rail, border, border, 4);
        rail.right = std::max<LONG>(rail.left + 1, x);
        rounded(dc, rail, accent, accent, 4);
        RECT knob{x - 7, y - 7, x + 8, y + 8};
        rounded(dc, knob, white, accent, 16);
        if (focus) {
            RECT ring = r;
            InflateRect(&ring, -1, -1);
            DrawFocusRect(dc, &ring);
        }
        value.clear();
    } else {
        foreground = c.title ? ink : (c.status ? RGB(157, 174, 198) : muted);
        if (value.find(L'\n') != std::wstring::npos || c.instructions || c.card)
            format = DT_WORDBREAK | DT_NOPREFIX;
        if (c.card) {
            RECT card = r;
            InflateRect(&card, -1, -1);
            rounded(dc, card, white, border);
            InflateRect(&content, -10, -7);
            foreground = ink;
        }
    }
    SetTextColor(dc, foreground);
    DrawTextW(dc, value.c_str(), static_cast<int>(value.size()), &content, format);
    BitBlt(target, 0, 0, r.right, r.bottom, dc, 0, 0, SRCCOPY);
    SelectObject(dc, oldFont);
    SelectObject(dc, oldBitmap);
    DeleteObject(bitmap);
    DeleteDC(dc);
}
inline LRESULT CALLBACK controlProc(HWND hwnd, UINT msg, WPARAM w, LPARAM l, UINT_PTR id, DWORD_PTR ref) {
    auto &c = *reinterpret_cast<Control *>(ref);
    if (!c.surface->highContrast) {
        if (msg == WM_ERASEBKGND)
            return 1;
        if (msg == WM_PAINT) {
            PAINTSTRUCT ps{};
            auto dc = BeginPaint(hwnd, &ps);
            paint(c, dc);
            EndPaint(hwnd, &ps);
            return 0;
        }
        if (msg == WM_PRINTCLIENT) {
            paint(c, reinterpret_cast<HDC>(w));
            return 0;
        }
    }
    if (msg == WM_MOUSEMOVE && !c.hover) {
        c.hover = true;
        TRACKMOUSEEVENT tracking{sizeof(tracking), TME_LEAVE, hwnd, 0};
        TrackMouseEvent(&tracking);
        InvalidateRect(hwnd, nullptr, FALSE);
    }
    if (msg == WM_MOUSELEAVE) {
        c.hover = false;
        InvalidateRect(hwnd, nullptr, FALSE);
    }
    if (msg == WM_SETFOCUS)
        reveal(c);
    if (msg == WM_MOUSEWHEEL && c.surface->content > c.surface->page &&
        !(c.kind == Kind::combo && SendMessageW(hwnd, CB_GETDROPPEDSTATE, 0, 0))) {
        scroll(*c.surface, -GET_WHEEL_DELTA_WPARAM(w) * 48 / WHEEL_DELTA);
        return 0;
    }
    if (msg == WM_NCDESTROY) {
        RemoveWindowSubclass(hwnd, controlProc, id);
        auto result = DefSubclassProc(hwnd, msg, w, l);
        delete &c;
        return result;
    }
    auto result = DefSubclassProc(hwnd, msg, w, l);
    switch (msg) {
    case WM_SETTEXT:
    case WM_SETFOCUS:
    case WM_KILLFOCUS:
    case WM_ENABLE:
    case WM_LBUTTONDOWN:
    case WM_LBUTTONUP:
    case WM_KEYDOWN:
    case WM_KEYUP:
    case BM_SETCHECK:
    case BM_SETSTATE:
    case CB_SETCURSEL:
    case TBM_SETPOS:
        InvalidateRect(hwnd, nullptr, FALSE);
        break;
    }
    return result;
}
inline LRESULT colors(Surface &s, HDC dc, bool list) {
    SetTextColor(dc, s.highContrast ? GetSysColor(COLOR_WINDOWTEXT) : ink);
    SetBkColor(dc, s.highContrast ? GetSysColor(COLOR_WINDOW) : (list ? white : background));
    return reinterpret_cast<LRESULT>(s.highContrast ? GetSysColorBrush(COLOR_WINDOW)
                                                    : (list ? s.inputBrush : s.panelBrush));
}
inline LRESULT CALLBACK panelProc(HWND hwnd, UINT msg, WPARAM w, LPARAM l) {
    auto *s = reinterpret_cast<Surface *>(GetWindowLongPtrW(hwnd, GWLP_USERDATA));
    if (msg == WM_NCCREATE) {
        s = static_cast<Surface *>(reinterpret_cast<CREATESTRUCTW *>(l)->lpCreateParams);
        SetWindowLongPtrW(hwnd, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(s));
    }
    if (!s)
        return DefWindowProcW(hwnd, msg, w, l);
    switch (msg) {
    case WM_COMMAND:
    case WM_HSCROLL:
    case WM_NOTIFY:
        return SendMessageW(s->root, msg, w, l);
    case WM_CTLCOLORSTATIC:
    case WM_CTLCOLORBTN:
    case WM_CTLCOLORLISTBOX:
        return colors(*s, reinterpret_cast<HDC>(w), msg == WM_CTLCOLORLISTBOX);
    case WM_ERASEBKGND:
        return 1;
    case WM_PAINT: {
        PAINTSTRUCT ps{};
        auto dc = BeginPaint(hwnd, &ps);
        RECT r{};
        GetClientRect(hwnd, &r);
        FillRect(dc, &r, s->highContrast ? GetSysColorBrush(COLOR_WINDOW) : s->panelBrush);
        line(dc, r.right - 1, 0, r.right - 1, r.bottom, border);
        EndPaint(hwnd, &ps);
        return 0;
    }
    case WM_MOUSEWHEEL:
        scroll(*s, -GET_WHEEL_DELTA_WPARAM(w) * 48 / WHEEL_DELTA);
        return 0;
    case WM_VSCROLL: {
        int next = s->offset;
        SCROLLINFO info{sizeof(info), SIF_TRACKPOS};
        GetScrollInfo(hwnd, SB_VERT, &info);
        switch (LOWORD(w)) {
        case SB_LINEUP:
            next -= 24;
            break;
        case SB_LINEDOWN:
            next += 24;
            break;
        case SB_PAGEUP:
            next -= s->page * 3 / 4;
            break;
        case SB_PAGEDOWN:
            next += s->page * 3 / 4;
            break;
        case SB_THUMBTRACK:
        case SB_THUMBPOSITION:
            next = info.nTrackPos;
            break;
        case SB_TOP:
            next = 0;
            break;
        case SB_BOTTOM:
            next = s->content;
            break;
        }
        s->offset = next;
        position(*s);
        return 0;
    }
    }
    return DefWindowProcW(hwnd, msg, w, l);
}
inline LRESULT CALLBACK rootProc(HWND hwnd, UINT msg, WPARAM w, LPARAM l, UINT_PTR id, DWORD_PTR ref) {
    auto &s = *reinterpret_cast<Surface *>(ref);
    if ((msg == WM_LBUTTONDOWN || msg == WM_RBUTTONDOWN) && GET_X_LPARAM(l) >= s.width)
        SetFocus(hwnd);
    if (msg == WM_CTLCOLORSTATIC || msg == WM_CTLCOLORBTN || msg == WM_CTLCOLORLISTBOX)
        return colors(s, reinterpret_cast<HDC>(w), msg == WM_CTLCOLORLISTBOX);
    if (msg == WM_SETTINGCHANGE || msg == WM_THEMECHANGED) {
        refreshContrast(s);
        RedrawWindow(hwnd, nullptr, nullptr, RDW_INVALIDATE | RDW_ALLCHILDREN);
    }
    auto result = DefSubclassProc(hwnd, msg, w, l);
    if (msg == WM_SIZE) {
        RECT r{};
        GetClientRect(hwnd, &r);
        MoveWindow(s.panel, 0, 76, s.width, std::max(1L, r.bottom - 76), TRUE);
        position(s);
    }
    if (msg == WM_NCDESTROY) {
        RemoveWindowSubclass(hwnd, rootProc, id);
        delete &s;
    }
    return result;
}
inline void apply(HWND root, int sidebarWidth, int oldWidth, const wchar_t *title, const wchar_t *subtitle) {
    auto *s = new Surface;
    s->root = root;
    s->width = sidebarWidth;
    refreshContrast(*s);
    auto font = [](int height, int weight) {
        return CreateFontW(height, 0, 0, 0, weight, FALSE, FALSE, FALSE, DEFAULT_CHARSET, OUT_DEFAULT_PRECIS,
                           CLIP_DEFAULT_PRECIS, CLEARTYPE_QUALITY, DEFAULT_PITCH, L"Segoe UI");
    };
    s->body = font(-15, FW_NORMAL);
    s->smallFont = font(-13, FW_NORMAL);
    s->heading = font(-22, FW_SEMIBOLD);
    s->strong = font(-15, FW_SEMIBOLD);
    s->panelBrush = CreateSolidBrush(background);
    s->inputBrush = CreateSolidBrush(white);
    SetWindowTextW(root, title);
    COLORREF caption = background, captionText = ink;
    DWM_WINDOW_CORNER_PREFERENCE corners = DWMWCP_ROUND;
    DwmSetWindowAttribute(root, DWMWA_WINDOW_CORNER_PREFERENCE, &corners, sizeof(corners));
    DwmSetWindowAttribute(root, DWMWA_CAPTION_COLOR, &caption, sizeof(caption));
    DwmSetWindowAttribute(root, DWMWA_TEXT_COLOR, &captionText, sizeof(captionText));
    std::vector<HWND> children;
    for (auto child = GetWindow(root, GW_CHILD); child; child = GetWindow(child, GW_HWNDNEXT))
        children.push_back(child);
    WNDCLASSW wc{};
    wc.lpfnWndProc = panelProc;
    wc.hInstance = GetModuleHandleW(nullptr);
    wc.lpszClassName = L"PhysicsLab.ModernPanel";
    wc.hCursor = LoadCursorW(nullptr, IDC_ARROW);
    RegisterClassW(&wc);
    RECT client{};
    GetClientRect(root, &client);
    s->panel = CreateWindowExW(WS_EX_CONTROLPARENT, wc.lpszClassName, L"Settings",
                               WS_CHILD | WS_VISIBLE | WS_CLIPCHILDREN | WS_VSCROLL, 0, 76, sidebarWidth,
                               std::max(1L, client.bottom - 76), root, nullptr, wc.hInstance, s);
    for (auto hwnd : children) {
        wchar_t cls[64]{};
        GetClassNameW(hwnd, cls, 64);
        auto *c = new Control;
        c->surface = s;
        c->hwnd = hwnd;
        RECT r{};
        GetWindowRect(hwnd, &r);
        MapWindowPoints(nullptr, root, reinterpret_cast<POINT *>(&r), 2);
        c->status = r.left >= sidebarWidth;
        c->title = !c->status && r.top < 35;
        c->subtitle = !c->status && r.top >= 35 && r.top < 76;
        auto style = GetWindowLongPtrW(hwnd, GWL_STYLE);
        if (lstrcmpiW(cls, L"BUTTON") == 0)
            c->kind = (style & BS_TYPEMASK) == BS_AUTOCHECKBOX ? Kind::check : Kind::button;
        else if (lstrcmpiW(cls, L"COMBOBOX") == 0)
            c->kind = Kind::combo;
        else if (lstrcmpiW(cls, TRACKBAR_CLASSW) == 0)
            c->kind = Kind::slider;
        else
            c->kind = Kind::label;
        auto value = text(hwnd);
        c->instructions = value.find(L"RMB") != value.npos || value.find(L"LMB") != value.npos ||
                          value.find(L"Space") != value.npos;
        c->card = c->kind == Kind::label && !c->status && !c->instructions && r.bottom - r.top >= 60;
        if (c->title)
            SetWindowTextW(hwnd, title);
        if (c->subtitle)
            SetWindowTextW(hwnd, subtitle);
        SetWindowTheme(hwnd, L"Explorer", nullptr);
        SendMessageW(hwnd, WM_SETFONT, reinterpret_cast<WPARAM>(c->title ? s->heading : s->body), FALSE);
        if (c->kind == Kind::combo) {
            SendMessageW(hwnd, CB_SETITEMHEIGHT, static_cast<WPARAM>(-1), 26);
            SendMessageW(hwnd, CB_SETITEMHEIGHT, 0, 26);
            r.bottom = r.top + 32;
        }
        if (c->kind == Kind::slider)
            SetWindowLongPtrW(hwnd, GWL_STYLE, style | TBS_NOTICKS);
        if (!c->status) {
            r.left = MulDiv(r.left, sidebarWidth, oldWidth);
            r.right = MulDiv(r.right, sidebarWidth, oldWidth);
            if (r.top >= 76) {
                r.top -= 76;
                r.bottom -= 76;
                SetParent(hwnd, s->panel);
            }
        }
        c->logical = r;
        SetWindowSubclass(hwnd, controlProc, 1, reinterpret_cast<DWORD_PTR>(c));
        s->controls.push_back(c);
        if (GetParent(hwnd) == s->panel)
            s->content = std::max(s->content, static_cast<int>(r.bottom + 12));
        else if (!c->status)
            MoveWindow(hwnd, r.left, r.top, r.right - r.left, r.bottom - r.top, TRUE);
    }
    // Reparenting changes sibling order. Restore a visual top-to-bottom Tab order.
    std::sort(s->controls.begin(), s->controls.end(), [](const Control *a, const Control *b) {
        return a->logical.top == b->logical.top ? a->logical.left < b->logical.left
                                                : a->logical.top < b->logical.top;
    });
    for (auto *c : s->controls)
        if (GetParent(c->hwnd) == s->panel)
            SetWindowPos(c->hwnd, HWND_BOTTOM, 0, 0, 0, 0, SWP_NOMOVE | SWP_NOSIZE | SWP_NOACTIVATE);
    SetWindowSubclass(root, rootProc, 1, reinterpret_cast<DWORD_PTR>(s));
    position(*s);
    RedrawWindow(root, nullptr, nullptr, RDW_INVALIDATE | RDW_ALLCHILDREN);
}
} // namespace modern
