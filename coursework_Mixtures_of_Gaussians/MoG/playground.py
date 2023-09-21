import numpy as np

#
# data = np.array([[1, 2, 9], [4, 5, 6], [7, 8, 9]])
# nDim, nData = np.shape(data)
#
# meanData = np.mean(data, 1)
# meanData = meanData.T
# # calculate covariance of data.  You should do this yourself to ensure you
# # understand how.  Check you have the right answer by comparing with the
# # numpy function np.cov().
# covData = np.zeros([nDim, nDim])
# for i in range(nDim):
#     for j in range(nDim):
#         for k in range(nData):
#             print(data[i][k])
#             print(meanData[i])
#             covData[i][j] = covData[i][j] + (data[i][k] - meanData[i]) * (data[j][k] - meanData[j])
#         covData[i][j] = covData[i][j] / (nData - 1)
# covData2 = np.cov(data)
#
# print(covData)
# print(covData2)
#
# a = np.array([np.array([1, 2, 3]), ] * 3).transpose()
# print(np.array([1, 2, 3]).shape)
# print(a)
# print(a.shape)

# im = plt.imread('bob_small.jpeg')
# # loading the segmentation mask
# gt = spio.loadmat('bob_GroundTruth_small.mat', squeeze_me=True)['gt']
# # %%
# # display the test image and the ground truth mask
# f, (ax1, ax2) = plt.subplots(1, 2)
# ax1.imshow(im)
# ax1.set_title('Image')
# ax2.imshow(gt)
# ax1.set_title('Ground Truth')
# plt.show()

# meanData = np.array([1, 2, 3])
# meanData.reshape([3, 1])
# print(meanData.shape)
# a = np.array([np.array(meanData), ] * 10).transpose()
# print(a)
# data = np.array([[1, 2, 3], [1, 2, 3]])
# data = data.reshape((3, 2))
# gaussMean = np.array([1, 2, 3])
# gaussCov = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
# print(data.shape)
# print(gaussMean.shape)
# print(gaussCov.shape)
# print((data - gaussMean).shape)
# print((data - gaussMean).T.shape)
#
# print(np.exp(
#     np.dot(np.dot((data - np.array([np.array(meanData), ] * 2).transpose()).transpose(), (np.linalg.inv(gaussCov))),
#            (data - gaussMean))))
# meanData = [1, 2, 3]
# nData = 100
# a = np.array([meanData, ] * nData).transpose()
# print(a)
# print(a.shape)
# mean = 1
# var = 4
# x = [2, 3, 4, 5, 6, 7]
# mean = np.squeeze(mean)
# var = np.squeeze(var)
# prob = np.exp(-0.5 * ((x - mean) ** 2) / (var))
# prob = prob / np.sqrt(2 * np.pi * var)
# print(prob)
# mixGaussTrue = dict()
# mixGaussTrue['k'] = 2  # number of gaussians
# mixGaussTrue['d'] = 1  # dimension of the data
# mixGaussTrue['weight'] = np.array([0.3, 0.7])  # weight assigned to each gaussian
# mixGaussTrue['mean'] = np.array([[-1, 1.5]])  # the mean for each gaussian
# mixGaussTrue['cov'] = np.reshape([0.5, 0.25], newshape=(1, 1, 2))  # the covariance for each gaussian
#
# a = mixGaussTrue
# print(a['cov'][0][0])
# a = np.array([[1, 2, 3], [4, 5, 6]])
# b = np.array([2, 3, 4])
# print(a * b)
# a = np.array([[1, 2, 3], [4, 5, 6], [7, 8, 9]])
# print(a)
# print(a[:, 1])
# print(a[1, :])

# a = np.array([2, 3])
# a = a.reshape(2, 1)
# # a.reshape((2, 1))
# print(a.shape)
# print(a.size)
a = np.zeros([2, 2])
b = np.zeros([4, 2])
print(a)
print(b)
a = np.vstack((a, b))

print(a)
