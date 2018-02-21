import numpy as np



### Function to perform custom Hough Transform in a Semi-vectorized manner:

def custom_hough(img_canny, houghVoteThresh, distThresh, segLengthThresh):

    # Range of rho is the diagonal of the image:
    [h,w]= img_canny.shape
    img_diag = np.sqrt(h**2 + w**2)



    # Create an accumulator matrix of the appropriate size:
    acc_h = int(img_diag/res_rho)  # Rows of the matrix
    acc_w = int(np.pi/res_theta)   # Columns of the matrix


    acc_mat = np.zeros((acc_h,acc_w))

    # Create a matrix to store the points which vote for each cell in the accumulator matrix:
    point_mat  = np.empty([acc_h,acc_w], dtype= np.object)

    # Now fill the matrix with empty list at each location:
    point_mat.fill([])

    # Override the broadcast: Need to understand this:
    point_mat = np.frompyfunc(list,1,1)(point_mat)


    # Find the indices where edge pixels are located:
    [y_edge, x_edge] = np.where(img_canny == 255)


    # Avoid calling the sin and cos functions repeatedly inside the loop:
    c_theta = np.cos(np.arange(0,np.pi, np.pi/135))
    s_theta = np.sin(np.arange(0,np.pi, np.pi/135))

    start_time = time.time()

    # Now loop over the edge pixels and get their votes:
    # Loop through and fill the accumulator matrix and store the points which vote for each bin:
    for x,y in zip(x_edge, y_edge):

        for theta_ind in np.arange(0,acc_w):

            rho_ind = int((x*c_theta[theta_ind] + y*s_theta[theta_ind])/res_rho)

            acc_mat[rho_ind,theta_ind] +=1

            point_mat[rho_ind, theta_ind].append((x,y))

    end_time = time.time()

    print ("Time to create accumulator matrix and store points is" , end_time - start_time)


    # Find the indices where the no of votes exceeds the min threshold:
    [rho_ex, theta_ex] = np.where(acc_mat > houghVoteThresh)

    # For the bins which met the criteria get the equations of the lines and the corresponding points which voted for that bin:
    lines =[]  # List of tuples to store the parameters (rho, theta) of each line
    list_points = []   # List of lists where each nested list contains tuples of the points which voted for each line


    st = time.time()
    for rho,theta in zip(rho_ex , theta_ex):

        lines.append((rho*res_rho , theta*res_theta))

        list_points.append(point_mat[rho,theta])
    et = time.time()


    print ("Time to collect info about lines and points is" ,et - st)


    # Get the number of lines:
    print (" The number of lines generated is : ")
    print (len(lines))


    ### Now extract the segments from the lines:

    mazeSegments = []


    st = time.time()
    for points, line in zip(list_points, lines):

        y_prime = np.zeros(len(points))

        count =0

        for point in points:

            # Rotate each of the points to the new co-ordinate system: (x', y')
            # We only need to find the rotated y' co-ordinates:
            # line[1] - stores the value of theta
            y_prime[count] = np.sin(line[1])*point[0] + np.cos(line[1])*point[1]
            count+=1


        # Add a sub-routine here to split the points belonging to a single line:

        # Sort the y' coordinates:
        ind_sort = np.argsort(y_prime)


        start = points[ind_sort[0]]

        for i in np.arange(1, len(ind_sort)):

            if abs(y_prime[ind_sort[i]] - y_prime[ind_sort[i-1]]) > distThresh:

                # Set the previous point as the end point:
                end = points[ind_sort[i-1]]

                # Compute the distance between the start and end point:
                currLength = np.sqrt( (end[0] - start[0])**2 + (end[1] - start[1])**2 )


                # Append the current segment: Only if the sgement is long enough:
                if currLength > segLengthThresh:
                    mazeSegments.append([start,end])


                # Set the current point as the start point of the new segment:
                start = points[ind_sort[i]]



        end = points[ind_sort[count-1]]


        mazeSegments.append([start,end])


    et = time.time()

    print ("Time to find the segments is" ,et - st)


    print ("The number of segments generated is: ")
    print(len(mazeSegments))
