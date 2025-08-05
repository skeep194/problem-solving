#define X real()
#define Y imag()
const double PI = acos(-1);
namespace Geometry
{
    template<typename T>
    T area(vector<complex<T>>& pos)
    {
        pos.push_back(pos[0]);
        T ret = 0;
        for(int i=0;i<pos.size()-1;++i)
        {
            ret += pos[i].X*pos[i+1].Y-pos[i+1].X*pos[i].Y;
        }
        pos.pop_back();
    return abs(ret);
    }

    template<typename T>
    T cross_product(const complex<T>& a, const complex<T>& b)
    {
        return (conj(a)*b).Y;
    }

    template<typename T>
    T inner_product(const complex<T>& a, const complex<T>& b)
    {
        return a.X*b.X+a.Y*b.Y;
    }

    template<typename T>
    int ccw(const complex<T> &a, const complex<T> &b, const complex<T> &pos)
    {
        T ret = cross_product(pos - a, pos - b);
        if (ret > 0)
            return 1;
        if (ret == 0)
            return 0;
        return -1;
        return ret;
    }

    template<typename T>
    T dist(const complex<T>& a, const complex<T>& b)
    {
        T x = a.X-b.X;
        T y = a.Y-b.Y;
        return x*x+y*y;
    }

    template<typename T>
    bool sort_xy(const complex<T>& a, const complex<T>& b)
    {
        return make_pair(a.X, a.Y) < make_pair(b.X, b.Y);
    }

    //0: not cross, 1: one cross and point is end of line, 2: one cross and point is not end of line, 3: many cross
    template<typename T>
    int line_cross(const pair<complex<T>, complex<T>>& a, const pair<complex<T>, complex<T>>& b)
    {
        T cp1 = ccw(a.first, a.second, b.first);
        T cp2 = ccw(a.first, a.second, b.second);
        T cp3 = ccw(b.first, b.second, a.first);
        T cp4 = ccw(b.first, b.second, a.second);
        if(cp1 == 0 && cp2 == 0 && cp3 == 0 && cp4 == 0)
        {
            pair<T, T> k1 = {a.first.X, a.first.Y};
            pair<T, T> k2 = {a.second.X, a.second.Y};
            pair<T, T> k3 = {b.first.X, b.first.Y};
            pair<T, T> k4 = {b.second.X, b.second.Y};
            if(k1 > k2) swap(k1, k2);
            if(k3 > k4) swap(k3, k4);
            if(k3 < k2 && k1 < k4)
            {
                return 3;
            }
            else if(k2 == k3 || k1 == k4)
            {
                return 1;
            }
            else
            {
                return 0;
            }
        }
        if(cp1*cp2 <= 0 && cp3*cp4 == 0 || cp1*cp2 == 0 && cp3*cp4 <= 0)
        {
            return 1;
        }
        if(cp1*cp2 < 0 && cp3*cp4 < 0)
            return 2;
        return 0;
    }

    template <typename T>
    vector<complex<T>> convex_hull(vector<complex<T>>& v)
    {
        sort(v.begin(), v.end(), [](const complex<T>& a, const complex<T>& b){
            return make_pair(a.X, a.Y) < make_pair(b.X, b.Y);
        });
        sort(v.begin()+1, v.end(), [&](const complex<T>& a, const complex<T>& b) {
            return make_pair(ccw(v[0], a, b), Geometry::dist(v[0], a)) < make_pair(ccw(v[0], b, a), Geometry::dist(v[0], b));
        });
        vector<complex<T>> s;
        for(int i=0;i<v.size();++i)
        {
            if(s.size() < 2)
                s.push_back(v[i]);
            else
            {
                while(s.size()>=2)
                {
                    auto here = s.back();
                    s.pop_back();
                    if(ccw(s.back(), here, v[i]) < 0)
                    {
                        s.push_back(here);
                        break;
                    }
                }
                s.push_back(v[i]);
            }
        }
        return s;
    }

    template <typename T>
    bool pos_in_convex(vector<complex<T>>& convex, complex<T>& pos)
    {
        int low = 1, high = convex.size()-1;
        if(ccw(convex[0], convex[low], pos) > 0) return false;
        if(ccw(convex[0], convex[high], pos) < 0) return false;
        int mid = (low+high)/2;
        while(low < high)
        {
            mid = (low + high) / 2;
            if(ccw(convex[0], convex[mid], pos) <= 0)
                low = mid + 1;
            else
                high = mid;
        }
        low--;
        return ccw(convex[low], convex[(low+1)%convex.size()], pos) <= 0;
    }
    ;
}
