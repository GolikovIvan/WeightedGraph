#define NOMINMAX
#include <windows.h>
#include <windowsx.h>
#include <commctrl.h>

#pragma comment(lib, "comctl32.lib")

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cwchar>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

struct Vertex
{
    int id;
    int x;
    int y;
};

struct Edge
{
    int from;
    int to;
    int weight;
};

struct Result
{
    bool ok = false;
    long long cost = 0;
    std::vector<int> path;
    std::wstring message;
};

struct TipItem
{
    // Храним данные элемента, у которого есть подсказка.
    HWND hwnd;
    WNDPROC oldProc;
    const wchar_t* text;
};

enum class Mode
{
    Normal,
    AddEdgeStart,
    AddEdgeEnd
};

const int WINDOW_W = 1000;
const int WINDOW_H = 700;
const int TOP_H = 72;
const int STATUS_H = 44;
const int VERTEX_R = 18;
const int MIN_VERTEX_DIST = 48;
const long long INF = std::numeric_limits<long long>::max() / 4;

const int ID_WEIGHT = 1001;
const int ID_ADD_EDGE = 1002;
const int ID_SET_START = 1003;
const int ID_SET_FINISH = 1004;
const int ID_DELETE_VERTEX = 1005;
const int ID_CLEAR = 1006;
const int ID_DIJKSTRA = 1007;
const int ID_BELLMAN = 1008;
const int ID_ASTAR = 1009;
const int ID_HELP = 1010;

HMENU ControlId(int id)
{
    return reinterpret_cast<HMENU>(static_cast<INT_PTR>(id));
}

HWND g_hwnd = nullptr;
HWND g_weightEdit = nullptr;
HWND g_tooltip = nullptr;
std::vector<TipItem> g_tips;
std::vector<Vertex> g_vertices;
std::vector<Edge> g_edges;
std::vector<int> g_resultPath;
std::wstring g_status = L"Клик по холсту добавляет вершину. Зажмите вершину мышью, чтобы перетащить.";
Mode g_mode = Mode::Normal;
int g_selected = -1;
int g_edgeStart = -1;
int g_start = -1;
int g_finish = -1;

// Эти поля нужны для ручного перетаскивания вершины.
bool g_dragging = false;
bool g_dragMoved = false;
int g_dragVertex = -1;
int g_dragDx = 0;
int g_dragDy = 0;

RECT CanvasRect()
{
    RECT r{ 0, TOP_H, WINDOW_W, WINDOW_H - STATUS_H };
    return r;
}

bool PointInCanvas(int x, int y)
{
    RECT r = CanvasRect();
    return x >= r.left && x <= r.right && y >= r.top && y <= r.bottom;
}

double Distance(int x1, int y1, int x2, int y2)
{
    double dx = double(x1 - x2);
    double dy = double(y1 - y2);
    return std::sqrt(dx * dx + dy * dy);
}

int FindVertexAt(int x, int y)
{
    for (int i = int(g_vertices.size()) - 1; i >= 0; --i)
    {
        const Vertex& v = g_vertices[i];
        if (Distance(x, y, v.x, v.y) <= VERTEX_R)
            return i;
    }
    return -1;
}

void ResetResult()
{
    g_resultPath.clear();
}

void Refresh()
{
    InvalidateRect(g_hwnd, nullptr, TRUE);
}

std::wstring PathToText(const std::vector<int>& path)
{
    std::wstringstream ss;
    for (size_t i = 0; i < path.size(); ++i)
    {
        if (i > 0)
            ss << L" -> ";
        ss << g_vertices[path[i]].id;
    }
    return ss.str();
}

void ShowResult(const Result& result)
{
    if (result.ok)
    {
        g_resultPath = result.path;
        std::wstringstream ss;
        ss << L"Путь: " << PathToText(result.path) << L", вес: " << result.cost;
        g_status = ss.str();
    }
    else
    {
        g_resultPath.clear();
        g_status = result.message;
    }
    Refresh();
}

bool HasNegativeEdge()
{
    for (const Edge& e : g_edges)
    {
        if (e.weight < 0)
            return true;
    }
    return false;
}

std::vector<int> RestorePath(const std::vector<int>& prev, int start, int finish)
{
    std::vector<int> path;
    int cur = finish;

    // Идем назад от финиша к старту по массиву prev.
    while (cur != -1)
    {
        path.push_back(cur);
        if (cur == start)
            break;
        cur = prev[cur];
    }

    if (path.back() != start)
        return {};

    std::reverse(path.begin(), path.end());
    return path;
}

bool CheckStartFinish(Result& result)
{
    if (g_start == -1 || g_finish == -1)
    {
        result.message = L"Выберите старт и финиш";
        return false;
    }

    if (g_start == g_finish)
    {
        result.ok = true;
        result.cost = 0;
        result.path = { g_start };
        return false;
    }

    return true;
}

Result Dijkstra()
{
    Result result;
    if (!CheckStartFinish(result))
        return result;

    if (HasNegativeEdge())
    {
        result.message = L"Алгоритм Дейкстры не работает с отрицательными весами";
        return result;
    }

    int n = int(g_vertices.size());
    std::vector<long long> dist(n, INF);
    std::vector<int> prev(n, -1);
    std::vector<bool> used(n, false);
    dist[g_start] = 0;

    // Каждый раз берем непосещенную вершину с самым малым расстоянием.
    for (int step = 0; step < n; ++step)
    {
        int v = -1;
        for (int i = 0; i < n; ++i)
        {
            if (!used[i] && (v == -1 || dist[i] < dist[v]))
                v = i;
        }

        if (v == -1 || dist[v] == INF)
            break;

        used[v] = true;
        for (const Edge& e : g_edges)
        {
            if (e.from == v && dist[v] + e.weight < dist[e.to])
            {
                dist[e.to] = dist[v] + e.weight;
                prev[e.to] = v;
            }
        }
    }

    if (dist[g_finish] == INF)
    {
        result.message = L"Путь не найден";
        return result;
    }

    result.ok = true;
    result.cost = dist[g_finish];
    result.path = RestorePath(prev, g_start, g_finish);
    return result;
}

Result BellmanFord()
{
    Result result;
    if (!CheckStartFinish(result))
        return result;

    int n = int(g_vertices.size());
    std::vector<long long> dist(n, INF);
    std::vector<int> prev(n, -1);
    dist[g_start] = 0;

    // Релаксация пробует улучшить путь через каждое ребро.
    for (int i = 0; i < n - 1; ++i)
    {
        bool changed = false;
        for (const Edge& e : g_edges)
        {
            if (dist[e.from] != INF && dist[e.from] + e.weight < dist[e.to])
            {
                dist[e.to] = dist[e.from] + e.weight;
                prev[e.to] = e.from;
                changed = true;
            }
        }
        if (!changed)
            break;
    }

    // Проверяем только циклы, до которых можно дойти из старта.
    for (const Edge& e : g_edges)
    {
        if (dist[e.from] != INF && dist[e.from] + e.weight < dist[e.to])
        {
            result.message = L"Найден отрицательный цикл от старта";
            return result;
        }
    }

    if (dist[g_finish] == INF)
    {
        result.message = L"Путь не найден";
        return result;
    }

    result.ok = true;
    result.cost = dist[g_finish];
    result.path = RestorePath(prev, g_start, g_finish);
    return result;
}

double HeuristicCoefficient()
{
    double coef = std::numeric_limits<double>::infinity();

    // Коэффициент делает эвристику A* безопасной для произвольных весов.
    for (const Edge& e : g_edges)
    {
        const Vertex& a = g_vertices[e.from];
        const Vertex& b = g_vertices[e.to];
        double geo = Distance(a.x, a.y, b.x, b.y);
        if (geo > 0.0001)
            coef = std::min(coef, double(e.weight) / geo);
    }

    if (!std::isfinite(coef) || coef <= 0.0)
        return 0.0;

    return coef;
}

double Heuristic(int v, double coef)
{
    const Vertex& a = g_vertices[v];
    const Vertex& b = g_vertices[g_finish];
    return Distance(a.x, a.y, b.x, b.y) * coef;
}

Result AStar()
{
    Result result;
    if (!CheckStartFinish(result))
        return result;

    if (HasNegativeEdge())
    {
        result.message = L"Алгоритм A* не работает с отрицательными весами";
        return result;
    }

    int n = int(g_vertices.size());
    double coef = HeuristicCoefficient();
    std::vector<long long> g(n, INF);
    std::vector<double> f(n, std::numeric_limits<double>::infinity());
    std::vector<int> prev(n, -1);
    std::vector<bool> closed(n, false);
    g[g_start] = 0;
    f[g_start] = Heuristic(g_start, coef);

    // A* выбирает вершину с минимальной оценкой f = g + h.
    for (int step = 0; step < n; ++step)
    {
        int v = -1;
        for (int i = 0; i < n; ++i)
        {
            if (!closed[i] && (v == -1 || f[i] < f[v]))
                v = i;
        }

        if (v == -1 || g[v] == INF)
            break;
        if (v == g_finish)
            break;

        closed[v] = true;
        for (const Edge& e : g_edges)
        {
            if (e.from != v)
                continue;

            long long nextG = g[v] + e.weight;
            if (nextG < g[e.to])
            {
                g[e.to] = nextG;
                f[e.to] = double(nextG) + Heuristic(e.to, coef);
                prev[e.to] = v;
            }
        }
    }

    if (g[g_finish] == INF)
    {
        result.message = L"Путь не найден";
        return result;
    }

    result.ok = true;
    result.cost = g[g_finish];
    result.path = RestorePath(prev, g_start, g_finish);
    return result;
}

bool ParseWeight(int& value)
{
    wchar_t buf[128];
    GetWindowTextW(g_weightEdit, buf, 128);
    std::wstring text = buf;

    if (text.empty() || text == L"-")
        return false;

    wchar_t* end = nullptr;
    errno = 0;
    long long parsed = std::wcstoll(text.c_str(), &end, 10);

    // end должен дойти до конца строки, иначе там лишний мусор.
    if (errno != 0 || end == text.c_str() || *end != L'\0')
        return false;

    if (parsed < std::numeric_limits<int>::min() || parsed > std::numeric_limits<int>::max())
        return false;

    value = int(parsed);
    return true;
}

void ClearGraph()
{
    if (g_dragging)
        ReleaseCapture();
    g_vertices.clear();
    g_edges.clear();
    g_resultPath.clear();
    g_selected = -1;
    g_edgeStart = -1;
    g_start = -1;
    g_finish = -1;
    g_mode = Mode::Normal;
    g_dragging = false;
    g_dragMoved = false;
    g_dragVertex = -1;
    g_status = L"Граф очищен";
    Refresh();
}

void DeleteSelectedVertex()
{
    if (g_selected < 0 || g_selected >= int(g_vertices.size()))
    {
        g_status = L"Сначала выберите вершину";
        Refresh();
        return;
    }

    int del = g_selected;
    if (g_dragging)
        ReleaseCapture();
    g_vertices.erase(g_vertices.begin() + del);

    std::vector<Edge> newEdges;
    for (Edge e : g_edges)
    {
        if (e.from == del || e.to == del)
            continue;
        if (e.from > del)
            --e.from;
        if (e.to > del)
            --e.to;
        newEdges.push_back(e);
    }
    g_edges = newEdges;

    // После удаления перенумеровываем вершины для простого вида.
    for (int i = 0; i < int(g_vertices.size()); ++i)
        g_vertices[i].id = i + 1;

    if (g_start == del) g_start = -1;
    else if (g_start > del) --g_start;

    if (g_finish == del) g_finish = -1;
    else if (g_finish > del) --g_finish;

    g_selected = -1;
    g_edgeStart = -1;
    g_mode = Mode::Normal;
    g_dragging = false;
    g_dragMoved = false;
    g_dragVertex = -1;
    ResetResult();
    g_status = L"Вершина удалена";
    Refresh();
}

bool TooCloseToOtherVertex(int x, int y, int skip = -1)
{
    for (int i = 0; i < int(g_vertices.size()); ++i)
    {
        if (i == skip)
            continue;
        const Vertex& v = g_vertices[i];
        if (Distance(x, y, v.x, v.y) < MIN_VERTEX_DIST)
            return true;
    }
    return false;
}

void AddOrUpdateEdge(int from, int to, int weight)
{
    if (from == to)
    {
        g_status = L"Петли запрещены";
        return;
    }

    for (Edge& e : g_edges)
    {
        if (e.from == from && e.to == to)
        {
            e.weight = weight;
            ResetResult();
            g_status = L"Вес ребра изменен";
            return;
        }
    }

    g_edges.push_back({ from, to, weight });
    ResetResult();
    g_status = L"Ребро добавлено";
}

void HandleCanvasClick(int x, int y)
{
    if (!PointInCanvas(x, y))
        return;

    int hit = FindVertexAt(x, y);

    // В режиме ребра клики выбирают начало и конец.
    if (g_mode == Mode::AddEdgeStart)
    {
        if (hit == -1)
        {
            g_status = L"Нужно выбрать вершину";
            Refresh();
            return;
        }
        g_edgeStart = hit;
        g_selected = hit;
        g_mode = Mode::AddEdgeEnd;
        g_status = L"Выберите конец ребра";
        Refresh();
        return;
    }

    if (g_mode == Mode::AddEdgeEnd)
    {
        if (hit == -1)
        {
            g_status = L"Нужно выбрать вершину";
            Refresh();
            return;
        }

        int weight = 0;
        if (!ParseWeight(weight))
        {
            g_status = L"Некорректный вес";
            Refresh();
            return;
        }

        AddOrUpdateEdge(g_edgeStart, hit, weight);
        g_selected = hit;
        g_edgeStart = -1;
        g_mode = Mode::Normal;
        Refresh();
        return;
    }

    if (hit != -1)
    {
        g_selected = hit;

        // Запоминаем вершину и смещение мыши от ее центра.
        g_dragging = true;
        g_dragMoved = false;
        g_dragVertex = hit;
        g_dragDx = x - g_vertices[hit].x;
        g_dragDy = y - g_vertices[hit].y;

        // Захват мыши нужен, чтобы отпускание не потерялось за окном.
        SetCapture(g_hwnd);
        g_status = L"Выбрана вершина: " + std::to_wstring(g_vertices[hit].id) + L". Можно перетащить мышью.";
        Refresh();
        return;
    }

    if (TooCloseToOtherVertex(x, y))
    {
        g_status = L"Вершины слишком близко";
        Refresh();
        return;
    }

    g_vertices.push_back({ int(g_vertices.size()) + 1, x, y });
    g_selected = int(g_vertices.size()) - 1;
    ResetResult();
    g_status = L"Вершина добавлена";
    Refresh();
}

void HandleMouseMove(int x, int y)
{
    if (!g_dragging || g_dragVertex < 0 || g_dragVertex >= int(g_vertices.size()))
        return;

    RECT r = CanvasRect();

    // Не даем вершине выйти за границы холста.
    int left = int(r.left) + VERTEX_R;
    int right = int(r.right) - VERTEX_R;
    int top = int(r.top) + VERTEX_R;
    int bottom = int(r.bottom) - VERTEX_R;
    int nx = std::max(left, std::min(x - g_dragDx, right));
    int ny = std::max(top, std::min(y - g_dragDy, bottom));

    // Не даем вершинам налезать друг на друга.
    if (TooCloseToOtherVertex(nx, ny, g_dragVertex))
    {
        g_status = L"Слишком близко к другой вершине";
        Refresh();
        return;
    }

    Vertex& v = g_vertices[g_dragVertex];
    if (v.x == nx && v.y == ny)
        return;

    // При переносе координаты меняются, поэтому старый путь уже не показываем.
    v.x = nx;
    v.y = ny;
    g_dragMoved = true;
    ResetResult();
    g_status = L"Вершина перемещается: " + std::to_wstring(v.id);
    Refresh();
}

void HandleMouseUp()
{
    if (!g_dragging)
        return;

    // Отпускание кнопки завершает перенос вершины.
    ReleaseCapture();
    g_dragging = false;

    if (g_dragMoved && g_dragVertex >= 0 && g_dragVertex < int(g_vertices.size()))
        g_status = L"Вершина перемещена: " + std::to_wstring(g_vertices[g_dragVertex].id);

    g_dragVertex = -1;
    g_dragMoved = false;
    Refresh();
}

bool IsEdgeInResultPath(int from, int to)
{
    for (size_t i = 1; i < g_resultPath.size(); ++i)
    {
        if (g_resultPath[i - 1] == from && g_resultPath[i] == to)
            return true;
    }
    return false;
}

void DrawArrow(HDC hdc, int x1, int y1, int x2, int y2)
{
    double angle = std::atan2(double(y2 - y1), double(x2 - x1));
    double endX = x2 - std::cos(angle) * VERTEX_R;
    double endY = y2 - std::sin(angle) * VERTEX_R;
    double startX = x1 + std::cos(angle) * VERTEX_R;
    double startY = y1 + std::sin(angle) * VERTEX_R;

    MoveToEx(hdc, int(startX), int(startY), nullptr);
    LineTo(hdc, int(endX), int(endY));

    // Маленькая стрелка показывает направление ребра.
    const double size = 10.0;
    double a1 = angle + 2.6;
    double a2 = angle - 2.6;
    POINT p[3]{
        { int(endX), int(endY) },
        { int(endX + std::cos(a1) * size), int(endY + std::sin(a1) * size) },
        { int(endX + std::cos(a2) * size), int(endY + std::sin(a2) * size) }
    };
    Polygon(hdc, p, 3);
}

void DrawGraph(HDC hdc)
{
    RECT canvas = CanvasRect();
    HBRUSH bg = CreateSolidBrush(RGB(250, 250, 250));
    FillRect(hdc, &canvas, bg);
    DeleteObject(bg);

    HPEN normalPen = CreatePen(PS_SOLID, 2, RGB(80, 80, 80));
    HPEN pathPen = CreatePen(PS_SOLID, 4, RGB(220, 60, 40));
    HBRUSH edgeBrush = CreateSolidBrush(RGB(80, 80, 80));
    HBRUSH pathBrush = CreateSolidBrush(RGB(220, 60, 40));

    SetBkMode(hdc, TRANSPARENT);

    for (const Edge& e : g_edges)
    {
        bool inPath = IsEdgeInResultPath(e.from, e.to);
        SelectObject(hdc, inPath ? pathPen : normalPen);
        SelectObject(hdc, inPath ? pathBrush : edgeBrush);

        const Vertex& a = g_vertices[e.from];
        const Vertex& b = g_vertices[e.to];
        DrawArrow(hdc, a.x, a.y, b.x, b.y);

        std::wstring w = std::to_wstring(e.weight);
        int mx = (a.x + b.x) / 2;
        int my = (a.y + b.y) / 2;
        TextOutW(hdc, mx + 5, my + 5, w.c_str(), int(w.size()));
    }

    DeleteObject(normalPen);
    DeleteObject(pathPen);
    DeleteObject(edgeBrush);
    DeleteObject(pathBrush);

    for (int i = 0; i < int(g_vertices.size()); ++i)
    {
        const Vertex& v = g_vertices[i];
        COLORREF color = RGB(245, 245, 245);
        if (i == g_selected)
            color = RGB(220, 235, 255);
        if (i == g_start)
            color = RGB(210, 255, 210);
        if (i == g_finish)
            color = RGB(255, 225, 225);

        HBRUSH brush = CreateSolidBrush(color);
        HPEN pen = CreatePen(PS_SOLID, 2, RGB(40, 40, 40));
        SelectObject(hdc, brush);
        SelectObject(hdc, pen);
        Ellipse(hdc, v.x - VERTEX_R, v.y - VERTEX_R, v.x + VERTEX_R, v.y + VERTEX_R);

        std::wstring text = std::to_wstring(v.id);
        RECT tr{ v.x - VERTEX_R, v.y - 9, v.x + VERTEX_R, v.y + 12 };
        DrawTextW(hdc, text.c_str(), -1, &tr, DT_CENTER | DT_SINGLELINE);

        if (i == g_start)
            TextOutW(hdc, v.x - VERTEX_R - 14, v.y - VERTEX_R - 12, L"S", 1);
        if (i == g_finish)
            TextOutW(hdc, v.x + VERTEX_R + 5, v.y - VERTEX_R - 12, L"F", 1);

        DeleteObject(brush);
        DeleteObject(pen);
    }

    HPEN border = CreatePen(PS_SOLID, 1, RGB(200, 200, 200));
    SelectObject(hdc, border);
    MoveToEx(hdc, canvas.left, canvas.top, nullptr);
    LineTo(hdc, canvas.right - 1, canvas.top);
    LineTo(hdc, canvas.right - 1, canvas.bottom - 1);
    LineTo(hdc, canvas.left, canvas.bottom - 1);
    LineTo(hdc, canvas.left, canvas.top);
    DeleteObject(border);
}

void DrawStatus(HDC hdc)
{
    RECT r{ 0, WINDOW_H - STATUS_H, WINDOW_W, WINDOW_H };
    HBRUSH bg = CreateSolidBrush(RGB(240, 240, 240));
    FillRect(hdc, &r, bg);
    DeleteObject(bg);

    SetBkMode(hdc, TRANSPARENT);
    RECT textRect{ 12, WINDOW_H - STATUS_H + 12, WINDOW_W - 12, WINDOW_H - 6 };
    DrawTextW(hdc, g_status.c_str(), -1, &textRect, DT_LEFT | DT_SINGLELINE | DT_END_ELLIPSIS);
}

void SetStartVertex()
{
    if (g_selected == -1)
    {
        g_status = L"Сначала выберите вершину";
        Refresh();
        return;
    }
    g_start = g_selected;
    ResetResult();
    g_status = L"Старт: " + std::to_wstring(g_vertices[g_start].id);
    Refresh();
}

void SetFinishVertex()
{
    if (g_selected == -1)
    {
        g_status = L"Сначала выберите вершину";
        Refresh();
        return;
    }
    g_finish = g_selected;
    ResetResult();
    g_status = L"Финиш: " + std::to_wstring(g_vertices[g_finish].id);
    Refresh();
}

void StartEdgeMode()
{
    if (g_dragging)
        ReleaseCapture();
    g_mode = Mode::AddEdgeStart;
    g_edgeStart = -1;
    g_dragging = false;
    g_dragMoved = false;
    g_dragVertex = -1;
    g_status = L"Выберите начало ребра";
    Refresh();
}

void CancelEdgeMode()
{
    if (g_mode != Mode::Normal)
    {
        g_mode = Mode::Normal;
        g_edgeStart = -1;
        g_status = L"Добавление ребра отменено";
        Refresh();
    }
}

void ShowHelp()
{
    const wchar_t* help =
        L"Краткая справка\n\n"
        L"1. Клик по пустому месту холста добавляет вершину.\n"
        L"2. Клик по вершине выбирает ее.\n"
        L"3. Зажмите вершину левой кнопкой мыши, чтобы перетащить ее.\n"
        L"4. Чтобы создать ребро, введите вес, нажмите 'Добавить ребро', выберите начало и конец.\n"
        L"5. Граф ориентированный: первая выбранная вершина -> вторая выбранная вершина.\n"
        L"6. Повторное такое же ребро обновляет его вес.\n"
        L"7. Выберите вершину и нажмите 'Старт' или 'Финиш'.\n"
        L"8. Найденный путь подсвечивается красным и выводится снизу.\n\n"
        L"Алгоритм Дейкстры ищет кратчайший путь, но не работает с отрицательными весами.\n"
        L"Алгоритм Беллмана-Форда умеет работать с отрицательными весами и находит отрицательный цикл от старта.\n"
        L"Алгоритм A* похож на алгоритм Дейкстры, но использует расстояние по холсту как безопасную подсказку.\n"
        L"Алгоритм A* также не запускается при отрицательных весах.";

    MessageBoxW(g_hwnd, help, L"Справка", MB_OK | MB_ICONINFORMATION);
}

TipItem* FindTip(HWND hwnd)
{
    // Ищем подсказку по handle элемента управления.
    for (TipItem& tip : g_tips)
    {
        if (tip.hwnd == hwnd)
            return &tip;
    }
    return nullptr;
}

LRESULT CALLBACK TipProc(HWND hwnd, UINT msg, WPARAM wParam, LPARAM lParam)
{
    TipItem* tip = FindTip(hwnd);

    // При наведении сразу показываем подсказку в нижней строке.
    if (tip && msg == WM_MOUSEMOVE)
    {
        if (g_status != tip->text)
        {
            g_status = tip->text;
            Refresh();
        }

        // Просим Windows сообщить, когда курсор уйдет с элемента.
        TRACKMOUSEEVENT tme{};
        tme.cbSize = sizeof(tme);
        tme.dwFlags = TME_LEAVE;
        tme.hwndTrack = hwnd;
        TrackMouseEvent(&tme);
    }

    if (tip)
        return CallWindowProcW(tip->oldProc, hwnd, msg, wParam, lParam);

    return DefWindowProcW(hwnd, msg, wParam, lParam);
}

void AddTooltip(HWND control, const wchar_t* text)
{
    // Добавляем обычную всплывающую подсказку Windows.
    TOOLINFOW ti{};
    ti.cbSize = sizeof(ti);
    ti.uFlags = TTF_IDISHWND | TTF_SUBCLASS;
    ti.hwnd = g_hwnd;
    ti.uId = reinterpret_cast<UINT_PTR>(control);
    ti.lpszText = const_cast<wchar_t*>(text);

    // Tooltip сам следит за наведением мыши на кнопку.
    SendMessageW(g_tooltip, TTM_ADDTOOLW, 0, reinterpret_cast<LPARAM>(&ti));

    // Подменяем обработчик, чтобы писать подсказку еще и в статус.
    WNDPROC oldProc = reinterpret_cast<WNDPROC>(
        SetWindowLongPtrW(control, GWLP_WNDPROC, reinterpret_cast<LONG_PTR>(TipProc)));
    g_tips.push_back({ control, oldProc, text });
}

void CreateControls(HWND hwnd)
{
    // Системный tooltip не требует своих окон и таймеров.
    g_tooltip = CreateWindowExW(WS_EX_TOPMOST, TOOLTIPS_CLASSW, nullptr,
        WS_POPUP | TTS_ALWAYSTIP | TTS_NOPREFIX,
        CW_USEDEFAULT, CW_USEDEFAULT, CW_USEDEFAULT, CW_USEDEFAULT,
        hwnd, nullptr, nullptr, nullptr);
    SendMessageW(g_tooltip, TTM_SETMAXTIPWIDTH, 0, 320);
    SendMessageW(g_tooltip, TTM_SETDELAYTIME, TTDT_INITIAL, 250);
    SendMessageW(g_tooltip, TTM_SETDELAYTIME, TTDT_AUTOPOP, 12000);

    CreateWindowExW(0, L"STATIC", L"Вес:", WS_CHILD | WS_VISIBLE,
        10, 14, 35, 20, hwnd, nullptr, nullptr, nullptr);

    g_weightEdit = CreateWindowExW(WS_EX_CLIENTEDGE, L"EDIT", L"1",
        WS_CHILD | WS_VISIBLE | ES_AUTOHSCROLL,
        45, 10, 70, 25, hwnd, ControlId(ID_WEIGHT), nullptr, nullptr);
    AddTooltip(g_weightEdit, L"Введите целый вес ребра. Можно 0 и отрицательные числа.");

    HWND addEdge = CreateWindowExW(0, L"BUTTON", L"Добавить ребро", WS_CHILD | WS_VISIBLE,
        125, 9, 115, 27, hwnd, ControlId(ID_ADD_EDGE), nullptr, nullptr);
    AddTooltip(addEdge, L"Включает режим ребра: выберите начало, потом конец. Esc отменяет.");

    HWND setStart = CreateWindowExW(0, L"BUTTON", L"Старт", WS_CHILD | WS_VISIBLE,
        250, 9, 65, 27, hwnd, ControlId(ID_SET_START), nullptr, nullptr);
    AddTooltip(setStart, L"Делает выбранную вершину началом пути.");

    HWND setFinish = CreateWindowExW(0, L"BUTTON", L"Финиш", WS_CHILD | WS_VISIBLE,
        322, 9, 70, 27, hwnd, ControlId(ID_SET_FINISH), nullptr, nullptr);
    AddTooltip(setFinish, L"Делает выбранную вершину концом пути.");

    HWND deleteVertex = CreateWindowExW(0, L"BUTTON", L"Удалить вершину", WS_CHILD | WS_VISIBLE,
        400, 9, 120, 27, hwnd, ControlId(ID_DELETE_VERTEX), nullptr, nullptr);
    AddTooltip(deleteVertex, L"Удаляет выбранную вершину и все ее ребра.");

    HWND clearGraph = CreateWindowExW(0, L"BUTTON", L"Очистить граф", WS_CHILD | WS_VISIBLE,
        528, 9, 110, 27, hwnd, ControlId(ID_CLEAR), nullptr, nullptr);
    AddTooltip(clearGraph, L"Полностью очищает холст и сбрасывает результат.");

    HWND dijkstra = CreateWindowExW(0, L"BUTTON", L"Дейкстра", WS_CHILD | WS_VISIBLE,
        650, 9, 82, 27, hwnd, ControlId(ID_DIJKSTRA), nullptr, nullptr);
    AddTooltip(dijkstra, L"Ищет кратчайший путь. Не работает с отрицательными весами.");

    HWND bellman = CreateWindowExW(0, L"BUTTON", L"Беллман-Форд", WS_CHILD | WS_VISIBLE,
        740, 9, 115, 27, hwnd, ControlId(ID_BELLMAN), nullptr, nullptr);
    AddTooltip(bellman, L"Ищет путь даже с отрицательными весами и проверяет циклы.");

    HWND astar = CreateWindowExW(0, L"BUTTON", L"A*", WS_CHILD | WS_VISIBLE,
        863, 9, 45, 27, hwnd, ControlId(ID_ASTAR), nullptr, nullptr);
    AddTooltip(astar, L"Ищет путь с безопасной подсказкой по расстоянию на холсте.");

    HWND help = CreateWindowExW(0, L"BUTTON", L"Справка", WS_CHILD | WS_VISIBLE,
        916, 9, 72, 27, hwnd, ControlId(ID_HELP), nullptr, nullptr);
    AddTooltip(help, L"Открывает краткое описание работы программы и алгоритмов.");

    CreateWindowExW(0, L"STATIC",
        L"Подсказка: вершины можно перетаскивать мышью. Во время действий смотрите нижнюю строку.",
        WS_CHILD | WS_VISIBLE,
        10, 45, 900, 20, hwnd, nullptr, nullptr, nullptr);
}

LRESULT CALLBACK WndProc(HWND hwnd, UINT msg, WPARAM wParam, LPARAM lParam)
{
    switch (msg)
    {
    case WM_CREATE:
        g_hwnd = hwnd;
        CreateControls(hwnd);
        return 0;

    case WM_LBUTTONDOWN:
        HandleCanvasClick(GET_X_LPARAM(lParam), GET_Y_LPARAM(lParam));
        return 0;

    case WM_MOUSEMOVE:
        // Пока кнопка зажата, двигаем выбранную вершину.
        HandleMouseMove(GET_X_LPARAM(lParam), GET_Y_LPARAM(lParam));
        return 0;

    case WM_LBUTTONUP:
        // Отпускание мыши завершает перетаскивание.
        HandleMouseUp();
        return 0;

    case WM_KEYDOWN:
        if (wParam == VK_ESCAPE)
            CancelEdgeMode();
        return 0;

    case WM_COMMAND:
        switch (LOWORD(wParam))
        {
        case ID_ADD_EDGE:
            StartEdgeMode();
            break;
        case ID_SET_START:
            SetStartVertex();
            break;
        case ID_SET_FINISH:
            SetFinishVertex();
            break;
        case ID_DELETE_VERTEX:
            DeleteSelectedVertex();
            break;
        case ID_CLEAR:
            ClearGraph();
            break;
        case ID_DIJKSTRA:
            ShowResult(Dijkstra());
            break;
        case ID_BELLMAN:
            ShowResult(BellmanFord());
            break;
        case ID_ASTAR:
            ShowResult(AStar());
            break;
        case ID_HELP:
            ShowHelp();
            break;
        }
        return 0;

    case WM_PAINT:
    {
        PAINTSTRUCT ps;
        HDC hdc = BeginPaint(hwnd, &ps);
        DrawGraph(hdc);
        DrawStatus(hdc);
        EndPaint(hwnd, &ps);
        return 0;
    }

    case WM_DESTROY:
        PostQuitMessage(0);
        return 0;
    }

    return DefWindowProcW(hwnd, msg, wParam, lParam);
}

int WINAPI wWinMain(HINSTANCE hInstance, HINSTANCE, PWSTR, int nCmdShow)
{
    const wchar_t CLASS_NAME[] = L"WeightedGraphWindow";

    INITCOMMONCONTROLSEX icc{};
    icc.dwSize = sizeof(icc);
    icc.dwICC = ICC_WIN95_CLASSES;
    InitCommonControlsEx(&icc);

    WNDCLASSW wc{};
    wc.lpfnWndProc = WndProc;
    wc.hInstance = hInstance;
    wc.lpszClassName = CLASS_NAME;
    wc.hCursor = LoadCursor(nullptr, IDC_ARROW);
    wc.hbrBackground = (HBRUSH)(COLOR_WINDOW + 1);

    RegisterClassW(&wc);

    RECT wr{ 0, 0, WINDOW_W, WINDOW_H };
    DWORD style = WS_OVERLAPPED | WS_CAPTION | WS_SYSMENU | WS_MINIMIZEBOX;
    AdjustWindowRect(&wr, style, FALSE);

    g_hwnd = CreateWindowExW(
        0,
        CLASS_NAME,
        L"Взвешенные графы",
        style,
        CW_USEDEFAULT,
        CW_USEDEFAULT,
        wr.right - wr.left,
        wr.bottom - wr.top,
        nullptr,
        nullptr,
        hInstance,
        nullptr);

    if (!g_hwnd)
        return 0;

    ShowWindow(g_hwnd, nCmdShow);
    UpdateWindow(g_hwnd);

    MSG msg{};
    while (GetMessageW(&msg, nullptr, 0, 0))
    {
        if (msg.message == WM_KEYDOWN && msg.wParam == VK_ESCAPE)
        {
            CancelEdgeMode();
            continue;
        }

        TranslateMessage(&msg);
        DispatchMessageW(&msg);
    }

    return 0;
}
