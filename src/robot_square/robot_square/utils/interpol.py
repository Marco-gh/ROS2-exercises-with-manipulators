def interpolator(A,B,n_camp):
    # Differenze con segno
    dx = B[0] - A[0]
    dy = B[1] - A[1]

    # Incrementi per campione
    incx = dx / n_camp
    incy = dy / n_camp

    # Generazione punti
    P = [A]
    for i in range(1, n_camp):
        P.append((A[0] + i * incx, A[1] + i * incy))
    # P.append(B)

    return P
