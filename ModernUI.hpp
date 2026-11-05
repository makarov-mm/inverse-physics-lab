#pragma once
// Flat, themed presentation for native Windows controls. The controls keep their keyboard, focus
// and accessibility behavior; only their painting is replaced, and the settings column becomes
// a scrollable panel. No external UI runtime is required.
//
// Usage: create the controls as children of the root window, then call modern::apply(root, options).
// Controls above Options::headerHeight stay on the root window as title/subtitle; controls right of
// the sidebar are treated as the status strip; everything else moves into the scrollable panel.
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
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
        canvas = RGB(3, 5, 8), statusText = RGB(157, 174, 198);
    inline constexpr COLORREF accentHover = RGB(32, 83, 201), accentPressed = RGB(24, 67, 170),
        buttonHover = RGB(234, 241, 255), buttonPressed = RGB(219, 231, 253),
        disabled = RGB(232, 237, 244);
    inline constexpr int wheelStep = 48, lineStep = 24, comboHeight = 32, comboDropHeight = 240;

    struct Options {
        int sidebarWidth = 320;          // final width of the settings column
        int designWidth = 320;           // width the controls were laid out for (scaled if different)
        int headerHeight = 76;           // title/subtitle area that stays above the scrollable panel
        const wchar_t* title = L"";      // replaces the text of the label found in the title area
        const wchar_t* subtitle = L"";   // replaces the text of the label found below the title
        int primaryButtonId = 0;         // control id drawn as the accent-colored primary button
    };

    struct Surface;
    enum class Kind { label, button, check, combo, slider };

    struct Control {
        Surface* surface{};
        HWND hwnd{};
        Kind kind{};
        RECT logical{}; // position in panel coordinates before scrolling
        bool hover{}, title{}, subtitle{}, status{}, card{}, instructions{};
    };

    struct Surface {
        HWND root{}, panel{};
        Options options;
        int offset{}, content{}, page{}; // scroll offset, content height and visible height in pixels
        bool highContrast{};
        HFONT body{}, smallFont{}, heading{}, strong{};
        HBRUSH panelBrush{}, inputBrush{};
        std::vector<Control*> controls;
        ~Surface() {
            for (auto font : { body, smallFont, heading, strong })
                DeleteObject(font);
            DeleteObject(panelBrush);
            DeleteObject(inputBrush);
        }
    };

    /// Owns a GDI object and restores the previously selected one on destruction.
    class Selected {
        HDC dc;
        HGDIOBJ object, previous;

    public:
        Selected(HDC target, HGDIOBJ created) : dc(target), object(created), previous(SelectObject(target, created)) {}
        ~Selected() {
            SelectObject(dc, previous);
            DeleteObject(object);
        }
        Selected(const Selected&) = delete;
        Selected& operator=(const Selected&) = delete;
    };

    inline std::wstring text(HWND hwnd) {
        std::wstring value(static_cast<size_t>(GetWindowTextLengthW(hwnd)) + 1, L'\0');
        value.resize(static_cast<size_t>(GetWindowTextW(hwnd, value.data(), static_cast<int>(value.size()))));
        return value;
    }
    inline void rounded(HDC dc, RECT r, COLORREF fill, COLORREF stroke, int radius = 8) {
        Selected brush(dc, CreateSolidBrush(fill)), pen(dc, CreatePen(PS_SOLID, 1, stroke));
        RoundRect(dc, r.left, r.top, r.right, r.bottom, radius, radius);
    }
    inline void line(HDC dc, int x1, int y1, int x2, int y2, COLORREF color, int width = 1) {
        Selected pen(dc, CreatePen(PS_SOLID, width, color));
        MoveToEx(dc, x1, y1, nullptr);
        LineTo(dc, x2, y2);
    }
    inline void refreshContrast(Surface& s) {
        HIGHCONTRASTW contrast{};
        contrast.cbSize = sizeof(contrast);
        SystemParametersInfoW(SPI_GETHIGHCONTRAST, sizeof(contrast), &contrast, 0);
        s.highContrast = (contrast.dwFlags & HCF_HIGHCONTRASTON) != 0;
    }
    inline bool inPanel(const Control& c) {
        return GetParent(c.hwnd) == c.surface->panel;
    }

    /// Applies the scroll offset: updates the scrollbar and moves every panel control.
    inline void position(Surface& s) {
        RECT client{};
        GetClientRect(s.panel, &client);
        s.page = client.bottom;
        s.offset = std::clamp(s.offset, 0, std::max(0, s.content - s.page));
        SCROLLINFO info{};
        info.cbSize = sizeof(info);
        info.fMask = SIF_RANGE | SIF_PAGE | SIF_POS;
        info.nMax = std::max(0, s.content - 1);
        info.nPage = static_cast<UINT>(s.page);
        info.nPos = s.offset;
        SetScrollInfo(s.panel, SB_VERT, &info, TRUE);
        GetClientRect(s.panel, &client); // the scrollbar may have changed the client width
        for (auto* c : s.controls) {
            if (!inPanel(*c))
                continue;
            const auto& r = c->logical;
            // Scale horizontally into the remaining client width so the scrollbar gutter never crops controls.
            const int left = MulDiv(r.left, client.right, s.options.sidebarWidth);
            const int right = MulDiv(r.right, client.right, s.options.sidebarWidth);
            SetWindowPos(c->hwnd, nullptr, left, r.top - s.offset, right - left,
                c->kind == Kind::combo ? comboDropHeight : r.bottom - r.top, SWP_NOZORDER | SWP_NOACTIVATE);
        }
        InvalidateRect(s.panel, nullptr, FALSE);
    }
    inline void scroll(Surface& s, int pixels) {
        s.offset += pixels;
        position(s);
    }
    /// Scrolls so that a (focused) control is fully visible.
    inline void reveal(Control& c) {
        auto& s = *c.surface;
        if (!inPanel(c))
            return;
        if (c.logical.top < s.offset)
            s.offset = c.logical.top;
        else if (c.logical.bottom > s.offset + s.page)
            s.offset = c.logical.bottom - s.page;
        position(s);
    }

    inline void paintButton(Control& c, HDC dc, RECT& content, COLORREF& foreground, UINT& format, bool enabled,
        bool focus) {
        const bool primary = GetDlgCtrlID(c.hwnd) == c.surface->options.primaryButtonId;
        const bool pressed = (SendMessageW(c.hwnd, BM_GETSTATE, 0, 0) & BST_PUSHED) != 0;
        COLORREF fill = primary ? accent : white;
        if (c.hover)
            fill = primary ? accentHover : buttonHover;
        if (pressed)
            fill = primary ? accentPressed : buttonPressed;
        if (!enabled)
            fill = disabled;
        InflateRect(&content, -1, -1);
        rounded(dc, content, fill, focus ? accent : (primary ? fill : border));
        if (focus) {
            RECT ring = content;
            InflateRect(&ring, -3, -3);
            DrawFocusRect(dc, &ring);
        }
        if (primary && enabled)
            foreground = white;
        format |= DT_CENTER;
    }
    inline void paintCheck(Control& c, HDC dc, RECT& content, const RECT& r, bool focus) {
        const bool checked = SendMessageW(c.hwnd, BM_GETCHECK, 0, 0) == BST_CHECKED;
        const int y = (r.bottom - 18) / 2;
        const RECT box{ 1, y, 19, y + 18 };
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
    }
    inline void paintCombo(Control& c, HDC dc, RECT& content, const RECT& r, std::wstring& value, bool focus) {
        InflateRect(&content, -1, -1);
        rounded(dc, content, white, focus || c.hover ? accent : border);
        const auto selection = SendMessageW(c.hwnd, CB_GETCURSEL, 0, 0);
        if (selection != CB_ERR) {
            const auto length = SendMessageW(c.hwnd, CB_GETLBTEXTLEN, static_cast<WPARAM>(selection), 0);
            if (length >= 0) {
                value.resize(static_cast<size_t>(length) + 1);
                SendMessageW(c.hwnd, CB_GETLBTEXT, static_cast<WPARAM>(selection), reinterpret_cast<LPARAM>(value.data()));
                value.resize(static_cast<size_t>(length));
            }
        }
        const int x = r.right - 18, y = r.bottom / 2; // chevron
        line(dc, x - 4, y - 2, x, y + 2, muted, 2);
        line(dc, x, y + 2, x + 4, y - 2, muted, 2);
        content.left += 10;
        content.right -= 32;
    }
    inline void paintSlider(Control& c, HDC dc, const RECT& r, bool focus) {
        RECT thumb{}, channel{};
        SendMessageW(c.hwnd, TBM_GETTHUMBRECT, 0, reinterpret_cast<LPARAM>(&thumb));
        SendMessageW(c.hwnd, TBM_GETCHANNELRECT, 0, reinterpret_cast<LPARAM>(&channel));
        const int x = (thumb.left + thumb.right) / 2, y = (thumb.top + thumb.bottom) / 2;
        RECT rail{ channel.left, y - 2, channel.right, y + 2 };
        rounded(dc, rail, border, border, 4);
        rail.right = std::max<LONG>(rail.left + 1, x);
        rounded(dc, rail, accent, accent, 4);
        const RECT knob{ x - 7, y - 7, x + 8, y + 8 };
        rounded(dc, knob, white, accent, 16);
        if (focus) {
            RECT ring = r;
            InflateRect(&ring, -1, -1);
            DrawFocusRect(dc, &ring);
        }
    }

    /// Paints one control into an off-screen bitmap and copies it to the target.
    inline void paint(Control& c, HDC target) {
        auto& s = *c.surface;
        RECT r{};
        GetClientRect(c.hwnd, &r);
        if (r.right <= 0 || r.bottom <= 0)
            return;
        HDC dc = CreateCompatibleDC(target);
        {
            Selected bitmap(dc, CreateCompatibleBitmap(target, r.right, r.bottom));
            const HBRUSH fill = CreateSolidBrush(c.status ? canvas : background);
            FillRect(dc, &r, fill);
            DeleteObject(fill);
            SetBkMode(dc, TRANSPARENT);
            HFONT font = c.title ? s.heading
                : (c.subtitle || c.instructions || c.status || c.card ? s.smallFont : s.body);
            if (c.kind == Kind::button)
                font = s.strong;
            const HGDIOBJ oldFont = SelectObject(dc, font);
            const bool enabled = IsWindowEnabled(c.hwnd) != FALSE;
            const bool focus = GetFocus() == c.hwnd;
            COLORREF foreground = enabled ? ink : muted;
            std::wstring value = text(c.hwnd);
            RECT content = r;
            UINT format = DT_SINGLELINE | DT_VCENTER | DT_END_ELLIPSIS | DT_NOPREFIX;
            switch (c.kind) {
            case Kind::button:
                paintButton(c, dc, content, foreground, format, enabled, focus);
                break;
            case Kind::check:
                paintCheck(c, dc, content, r, focus);
                break;
            case Kind::combo:
                paintCombo(c, dc, content, r, value, focus);
                break;
            case Kind::slider:
                paintSlider(c, dc, r, focus);
                value.clear();
                break;
            case Kind::label:
                foreground = c.title ? ink : (c.status ? statusText : muted);
                if (value.find(L'\n') != std::wstring::npos || c.instructions || c.card)
                    format = DT_WORDBREAK | DT_NOPREFIX;
                if (c.card) {
                    RECT card = r;
                    InflateRect(&card, -1, -1);
                    rounded(dc, card, white, border);
                    InflateRect(&content, -10, -7);
                    foreground = ink;
                }
                break;
            }
            SetTextColor(dc, foreground);
            DrawTextW(dc, value.c_str(), static_cast<int>(value.size()), &content, format);
            BitBlt(target, 0, 0, r.right, r.bottom, dc, 0, 0, SRCCOPY);
            SelectObject(dc, oldFont);
        }
        DeleteDC(dc);
    }

    inline LRESULT CALLBACK controlProc(HWND hwnd, UINT msg, WPARAM w, LPARAM l, UINT_PTR id, DWORD_PTR ref) {
        auto& c = *reinterpret_cast<Control*>(ref);
        if (!c.surface->highContrast) {
            if (msg == WM_ERASEBKGND)
                return 1;
            if (msg == WM_PAINT) {
                PAINTSTRUCT ps{};
                const HDC dc = BeginPaint(hwnd, &ps);
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
            TRACKMOUSEEVENT tracking{};
            tracking.cbSize = sizeof(tracking);
            tracking.dwFlags = TME_LEAVE;
            tracking.hwndTrack = hwnd;
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
            scroll(*c.surface, -GET_WHEEL_DELTA_WPARAM(w) * wheelStep / WHEEL_DELTA);
            return 0;
        }
        if (msg == WM_NCDESTROY) {
            RemoveWindowSubclass(hwnd, controlProc, id);
            const auto result = DefSubclassProc(hwnd, msg, w, l);
            delete& c; // allocated in apply(); the control's lifetime ends here
            return result;
        }
        const auto result = DefSubclassProc(hwnd, msg, w, l);
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

    inline LRESULT colors(Surface& s, HDC dc, bool list) {
        SetTextColor(dc, s.highContrast ? GetSysColor(COLOR_WINDOWTEXT) : ink);
        SetBkColor(dc, s.highContrast ? GetSysColor(COLOR_WINDOW) : (list ? white : background));
        return reinterpret_cast<LRESULT>(s.highContrast ? GetSysColorBrush(COLOR_WINDOW)
            : (list ? s.inputBrush : s.panelBrush));
    }

    inline LRESULT CALLBACK panelProc(HWND hwnd, UINT msg, WPARAM w, LPARAM l) {
        auto* s = reinterpret_cast<Surface*>(GetWindowLongPtrW(hwnd, GWLP_USERDATA));
        if (msg == WM_NCCREATE) {
            s = static_cast<Surface*>(reinterpret_cast<CREATESTRUCTW*>(l)->lpCreateParams);
            SetWindowLongPtrW(hwnd, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(s));
        }
        if (!s)
            return DefWindowProcW(hwnd, msg, w, l);
        switch (msg) {
        case WM_COMMAND:
        case WM_HSCROLL:
        case WM_NOTIFY:
            return SendMessageW(s->root, msg, w, l); // the application handles its controls on the root
        case WM_CTLCOLORSTATIC:
        case WM_CTLCOLORBTN:
        case WM_CTLCOLORLISTBOX:
            return colors(*s, reinterpret_cast<HDC>(w), msg == WM_CTLCOLORLISTBOX);
        case WM_ERASEBKGND:
            return 1;
        case WM_PAINT: {
            PAINTSTRUCT ps{};
            const HDC dc = BeginPaint(hwnd, &ps);
            RECT r{};
            GetClientRect(hwnd, &r);
            FillRect(dc, &r, s->highContrast ? GetSysColorBrush(COLOR_WINDOW) : s->panelBrush);
            line(dc, r.right - 1, 0, r.right - 1, r.bottom, border);
            EndPaint(hwnd, &ps);
            return 0;
        }
        case WM_MOUSEWHEEL:
            scroll(*s, -GET_WHEEL_DELTA_WPARAM(w) * wheelStep / WHEEL_DELTA);
            return 0;
        case WM_VSCROLL: {
            int next = s->offset;
            SCROLLINFO info{};
            info.cbSize = sizeof(info);
            info.fMask = SIF_TRACKPOS;
            GetScrollInfo(hwnd, SB_VERT, &info);
            switch (LOWORD(w)) {
            case SB_LINEUP:
                next -= lineStep;
                break;
            case SB_LINEDOWN:
                next += lineStep;
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
        auto& s = *reinterpret_cast<Surface*>(ref);
        if ((msg == WM_LBUTTONDOWN || msg == WM_RBUTTONDOWN) && GET_X_LPARAM(l) >= s.options.sidebarWidth)
            SetFocus(hwnd); // clicking the canvas takes focus away from the controls
        if (msg == WM_CTLCOLORSTATIC || msg == WM_CTLCOLORBTN || msg == WM_CTLCOLORLISTBOX)
            return colors(s, reinterpret_cast<HDC>(w), msg == WM_CTLCOLORLISTBOX);
        if (msg == WM_SETTINGCHANGE || msg == WM_THEMECHANGED) {
            refreshContrast(s);
            RedrawWindow(hwnd, nullptr, nullptr, RDW_INVALIDATE | RDW_ALLCHILDREN);
        }
        const auto result = DefSubclassProc(hwnd, msg, w, l);
        if (msg == WM_SIZE) {
            RECT r{};
            GetClientRect(hwnd, &r);
            const int header = s.options.headerHeight;
            MoveWindow(s.panel, 0, header, s.options.sidebarWidth, std::max(1L, r.bottom - header), TRUE);
            position(s);
        }
        if (msg == WM_NCDESTROY) {
            RemoveWindowSubclass(hwnd, rootProc, id);
            delete& s; // allocated in apply(); the root window's lifetime ends here
        }
        return result;
    }

    inline Kind classify(HWND hwnd, LONG_PTR style) {
        wchar_t cls[64]{};
        GetClassNameW(hwnd, cls, 64);
        if (lstrcmpiW(cls, L"BUTTON") == 0)
            return (style & BS_TYPEMASK) == BS_AUTOCHECKBOX ? Kind::check : Kind::button;
        if (lstrcmpiW(cls, L"COMBOBOX") == 0)
            return Kind::combo;
        if (lstrcmpiW(cls, TRACKBAR_CLASSW) == 0)
            return Kind::slider;
        return Kind::label;
    }

    /// Restyles the root window and every child control created so far. Call once, after the controls exist.
    inline void apply(HWND root, const Options& options) {
        auto* s = new Surface; // owned by the root window subclass, released on WM_NCDESTROY
        s->root = root;
        s->options = options;
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
        SetWindowTextW(root, options.title);
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
        const int header = options.headerHeight, sidebar = options.sidebarWidth;
        s->panel = CreateWindowExW(WS_EX_CONTROLPARENT, wc.lpszClassName, L"Settings",
            WS_CHILD | WS_VISIBLE | WS_CLIPCHILDREN | WS_VSCROLL, 0, header, sidebar,
            std::max(1L, client.bottom - header), root, nullptr, wc.hInstance, s);

        for (auto hwnd : children) {
            auto* c = new Control; // owned by the control's subclass, released on WM_NCDESTROY
            c->surface = s;
            c->hwnd = hwnd;
            RECT r{};
            GetWindowRect(hwnd, &r);
            MapWindowPoints(nullptr, root, reinterpret_cast<POINT*>(&r), 2);
            // Role by placement: right of the sidebar is the status strip, the header holds title and subtitle.
            c->status = r.left >= sidebar;
            c->title = !c->status && r.top < header / 2;
            c->subtitle = !c->status && r.top >= header / 2 && r.top < header;
            const auto style = GetWindowLongPtrW(hwnd, GWL_STYLE);
            c->kind = classify(hwnd, style);
            const std::wstring value = text(hwnd);
            c->instructions = value.find(L"RMB") != value.npos || value.find(L"LMB") != value.npos ||
                value.find(L"Space") != value.npos;
            c->card = c->kind == Kind::label && !c->status && !c->instructions && r.bottom - r.top >= 60;
            if (c->title)
                SetWindowTextW(hwnd, options.title);
            if (c->subtitle)
                SetWindowTextW(hwnd, options.subtitle);
            SetWindowTheme(hwnd, L"Explorer", nullptr);
            SendMessageW(hwnd, WM_SETFONT, reinterpret_cast<WPARAM>(c->title ? s->heading : s->body), FALSE);
            if (c->kind == Kind::combo) {
                SendMessageW(hwnd, CB_SETITEMHEIGHT, static_cast<WPARAM>(-1), 26);
                SendMessageW(hwnd, CB_SETITEMHEIGHT, 0, 26);
                r.bottom = r.top + comboHeight;
            }
            if (c->kind == Kind::slider)
                SetWindowLongPtrW(hwnd, GWL_STYLE, style | TBS_NOTICKS);
            if (!c->status) {
                r.left = MulDiv(r.left, sidebar, options.designWidth);
                r.right = MulDiv(r.right, sidebar, options.designWidth);
                if (r.top >= header) {
                    r.top -= header;
                    r.bottom -= header;
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
        std::sort(s->controls.begin(), s->controls.end(), [](const Control* a, const Control* b) {
            return a->logical.top == b->logical.top ? a->logical.left < b->logical.left
                : a->logical.top < b->logical.top;
            });
        for (auto* c : s->controls)
            if (inPanel(*c))
                SetWindowPos(c->hwnd, HWND_BOTTOM, 0, 0, 0, 0, SWP_NOMOVE | SWP_NOSIZE | SWP_NOACTIVATE);
        SetWindowSubclass(root, rootProc, 1, reinterpret_cast<DWORD_PTR>(s));
        position(*s);
        RedrawWindow(root, nullptr, nullptr, RDW_INVALIDATE | RDW_ALLCHILDREN);
    }
}
