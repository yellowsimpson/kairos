 # 레이브링을 이용하여 바운딩 박스 표시
    cnt, _, stats, center = cv2.connectedComponentsWithStats(diff)
    
    #retval, labels, stats, centroids = cv2.connectedComponentsWithStats(src, connectivity=8, ltype=cv2.CV_32S)
