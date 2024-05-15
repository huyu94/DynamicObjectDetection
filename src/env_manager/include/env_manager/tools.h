double updateMean(double x_mean, double x_new,int n)
{
    return x_mean + (x_new - x_mean) / n;
}