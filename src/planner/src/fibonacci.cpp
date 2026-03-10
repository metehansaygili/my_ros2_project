#include <bits/stdc++.h>

int dp[1000]={-1};
int fibonacci(int n){
    if(n <= 1) return n;
    if(dp[n] != -1) return dp[n];
    return dp[n]=fibonacci(n-1) + fibonacci(n-2);
}

int main(){
    int n;
    

}