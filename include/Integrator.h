#ifndef CODEPAMI_INTEGRATOR_H
#define CODEPAMI_INTEGRATOR_H

template <typename Func>
// Implement the generalized RK4 method
void runge_kutta_4(PRECISION_DATA_TYPE t0, PRECISION_DATA_TYPE* y0, int n, PRECISION_DATA_TYPE t_end, PRECISION_DATA_TYPE h, Func f) {
    PRECISION_DATA_TYPE t = t0; // Current time
    PRECISION_DATA_TYPE* y = new PRECISION_DATA_TYPE[n]; // Current state vector
    PRECISION_DATA_TYPE* k1 = new PRECISION_DATA_TYPE[n];
    PRECISION_DATA_TYPE* k2 = new PRECISION_DATA_TYPE[n];
    PRECISION_DATA_TYPE* k3 = new PRECISION_DATA_TYPE[n];
    PRECISION_DATA_TYPE* k4 = new PRECISION_DATA_TYPE[n];
    PRECISION_DATA_TYPE* temp = new PRECISION_DATA_TYPE[n]; // Temporary state vector

    // Initialize y with the initial condition y0
    for (int i = 0; i < n; ++i) {
        y[i] = y0[i];
    }

    // Main integration loop
    while (t < t_end) {
        // Compute k1
        f(t, y, k1, n);
        for (int i = 0; i < n; ++i) {
            k1[i] *= h;
            temp[i] = y[i] + k1[i] / 2.0;
        }

        // Compute k2
        f(t + h / 2.0, temp, k2, n);
        for (int i = 0; i < n; ++i) {
            k2[i] *= h;
            temp[i] = y[i] + k2[i] / 2.0;
        }

        // Compute k3
        f(t + h / 2.0, temp, k3, n);
        for (int i = 0; i < n; ++i) {
            k3[i] *= h;
            temp[i] = y[i] + k3[i];
        }

        // Compute k4
        f(t + h, temp, k4, n);
        for (int i = 0; i < n; ++i) {
            k4[i] *= h;
        }

        // Update y
        for (int i = 0; i < n; ++i) {
            y[i] += (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]) / 6.0;
        }

        // Update time
        t += h;
    }
    for (int i = 0; i < n; ++i) {
        y0[i] = y[i];
    }
    delete[] y;
    delete[] k1;
    delete[] k2;
    delete[] k3;
    delete[] k4;
    delete[] temp;
}


template <typename Func>
PRECISION_DATA_TYPE runge_kutta_4_maximized(PRECISION_DATA_TYPE t0, PRECISION_DATA_TYPE* y0, int n, PRECISION_DATA_TYPE t_end, PRECISION_DATA_TYPE h, Func f, const PRECISION_DATA_TYPE* lookup_length) {
    PRECISION_DATA_TYPE t = t0; // Current time
    PRECISION_DATA_TYPE* y = new PRECISION_DATA_TYPE[n]; // Current state vector
    PRECISION_DATA_TYPE* k1 = new PRECISION_DATA_TYPE[n];
    PRECISION_DATA_TYPE* k2 = new PRECISION_DATA_TYPE[n];
    PRECISION_DATA_TYPE* k3 = new PRECISION_DATA_TYPE[n];
    PRECISION_DATA_TYPE* k4 = new PRECISION_DATA_TYPE[n];
    PRECISION_DATA_TYPE* temp = new PRECISION_DATA_TYPE[n]; // Temporary state vector

    for (int i = 0; i < n; ++i) {
        y[i] = y0[i];
    }

    // Main integration loop
    while (t < t_end) {
        int j = 0;
        for(int i = 0; i < n; i++){
            if(y[i] > lookup_length[i])
                j++;
        }
        if(j == n){
            for (int i = 0; i < n; ++i) {
                y0[i] = y[i];
            }
            delete[] y;
            delete[] k1;
            delete[] k2;
            delete[] k3;
            delete[] k4;
            delete[] temp;
            return t;
        }
        // Compute k1
        f(t, y, k1, n);
        for (int i = 0; i < n; ++i) {
            k1[i] *= h;
            temp[i] = y[i] + k1[i] / 2.0;
        }

        // Compute k2
        f(t + h / 2.0, temp, k2, n);
        for (int i = 0; i < n; ++i) {
            k2[i] *= h;
            temp[i] = y[i] + k2[i] / 2.0;
        }

        // Compute k3
        f(t + h / 2.0, temp, k3, n);
        for (int i = 0; i < n; ++i) {
            k3[i] *= h;
            temp[i] = y[i] + k3[i];
        }

        // Compute k4
        f(t + h, temp, k4, n);
        for (int i = 0; i < n; ++i) {
            k4[i] *= h;
        }

        // Update y
        for (int i = 0; i < n; ++i) {
            y[i] += (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]) / 6.0;
        }

        // Update time
        t += h;
    }
    for (int i = 0; i < n; ++i) {
        y0[i] = y[i];
    }
    delete[] y;
    delete[] k1;
    delete[] k2;
    delete[] k3;
    delete[] k4;
    delete[] temp;
    return t_end;
}

#endif //CODEPAMI_INTEGRATOR_H
