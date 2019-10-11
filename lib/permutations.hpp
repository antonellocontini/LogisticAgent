// CPP program to print all permutations using 
// Johnson and Trotter algorithm. 
#include <bits/stdc++.h> 

template<class T>
class perm_iterator
{
private:
    const bool LEFT_TO_RIGHT = true;
    const bool RIGHT_TO_LEFT = false;
    std::vector<T> elements, result;
    std::vector<int> indices;
    std::vector<bool> dir;
    int n, n_perms, count;

    int fact(int n) 
    { 
        int res = 1; 
        for (int i = 1; i <= n; i++) 
            res = res * i; 
        return res; 
    }

    int searchArr(int mobile) 
    { 
        for (int i = 0; i < n; i++) 
            if (indices[i] == mobile) 
            return i + 1; 
    } 
    
    // To carry out step 1 of the algorithm i.e. 
    // to find the largest mobile integer.
    int getMobile() 
    { 
        int mobile_prev = 0, mobile = 0; 
        for (int i = 0; i < n; i++) 
        { 
            // direction 0 represents RIGHT TO LEFT. 
            if (dir[indices[i]-1] == RIGHT_TO_LEFT && i!=0) 
            { 
                if (indices[i] > indices[i-1] && indices[i] > mobile_prev) 
                { 
                    mobile = indices[i]; 
                    mobile_prev = mobile; 
                } 
            } 
    
            // direction 1 represents LEFT TO RIGHT. 
            if (dir[indices[i]-1] == LEFT_TO_RIGHT && i!=n-1) 
            { 
                if (indices[i] > indices[i+1] && indices[i] > mobile_prev) 
                { 
                    mobile = indices[i]; 
                    mobile_prev = mobile; 
                } 
            } 
        } 
    
        if (mobile == 0 && mobile_prev == 0) 
            return 0; 
        else
            return mobile; 
    } 

    void getOnePerm()
    {
        int mobile = getMobile(); 
        int pos = searchArr(mobile); 
    
        // swapping the elements according to the 
        // direction i.e. dir[]. 
        if (dir[indices[pos - 1] - 1] ==  RIGHT_TO_LEFT) 
        std::swap(indices[pos-1], indices[pos-2]); 
    
        else if (dir[indices[pos - 1] - 1] == LEFT_TO_RIGHT) 
        std::swap(indices[pos], indices[pos-1]); 
    
        // changing the directions for elements 
        // greater than largest mobile integer. 
        for (int i = 0; i < n; i++) 
        { 
            if (indices[i] > mobile) 
            { 
                if (dir[indices[i] - 1] == LEFT_TO_RIGHT) 
                    dir[indices[i] - 1] = RIGHT_TO_LEFT; 
                else if (dir[indices[i] - 1] == RIGHT_TO_LEFT) 
                    dir[indices[i] - 1] = LEFT_TO_RIGHT; 
            } 
        } 
    }
public:
    perm_iterator(const std::vector<T> &elements) : elements(elements),
                                                    result(elements),
                                                    indices(elements.size(), 0),
                                                    dir(elements.size(), RIGHT_TO_LEFT),
                                                    n(elements.size()),
                                                    n_perms(fact(n)),
                                                    count(0)
    {
        for(int i=0; i<n; i++)
        {
            indices[i] = i + 1;
        }
    }

    std::vector<T>& operator*()
    {
        for(int i=0; i<indices.size(); i++)
        {
            result[i] = elements[indices[i]-1];
        }
        return result;
    }

    perm_iterator& operator++()
    {
        if (count < n_perms - 1)
        {
            getOnePerm();
            count++;
        }
        else
        {
            throw std::overflow_error("Fine permutazioni");
        }
    }
};