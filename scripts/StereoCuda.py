import numpy as np
import cv2
from cv2 import cuda
class StereoCuda:
    """
    This class takes care of the CUDA input such that such that images
    can be provided as numpy array
    """
    def __init__(self,
                 num_disparities: int = 128,
                 block_size: int = 25,
                 bp_ndisp: int = 64,
                 min_disparity: int = 16,
                 uniqueness_ratio: int = 5
                 ) -> None:
        self.stereo_bm_cuda = cuda.createStereoBM(numDisparities=num_disparities,
                                                  blockSize=block_size)
        self.stereo_bp_cuda = cuda.createStereoBeliefPropagation(ndisp=bp_ndisp)
        self.stereo_bcp_cuda = cuda.createStereoConstantSpaceBP(min_disparity)
        self.stereo_sgm_cuda = cuda.createStereoSGM(minDisparity=min_disparity,
                                                    numDisparities=num_disparities,
                                                    uniquenessRatio=uniqueness_ratio
                                                    )
    @staticmethod
    def __numpy_to_gpumat(np_image: np.ndarray) -> cv2.cuda_GpuMat:
        """
        This method converts the numpy image matrix to a matrix that
        can be used by opencv cuda.
        Args:
            np_image: the numpy image matrix
        Returns:
            The image as a cuda matrix
        """
        image_cuda = cv2.cuda_GpuMat()
        image_cuda.upload(cv2.cvtColor(np_image, cv2.COLOR_BGR2GRAY))
        return image_cuda
    def compute_disparity(self, left_img: np.ndarray,
                          right_img: np.ndarray,
                          algorithm_name: str = "stereo_sgm_cuda",
                         color=True) -> np.ndarray:
        """
        Computes the disparity map using the named algorithm.
        Args:
            left_img: the numpy image matrix for the left camera
            right_img: the numpy image matrix for the right camera
            algorithm_name: the algorithm to use for calculating the disparity map
        Returns:
            The disparity map
        """
        algorithm = getattr(self, algorithm_name)
        left_cuda = self.__numpy_to_gpumat(left_img)
        right_cuda = self.__numpy_to_gpumat(right_img)
        if algorithm_name == "stereo_sgm_cuda":
            disparity_sgm_cuda_2 = cv2.cuda_GpuMat()
            disparity_sgm_cuda_1 = algorithm.compute(left_cuda,
                                                     right_cuda,
                                                     disparity_sgm_cuda_2)
            return disparity_sgm_cuda_1.download()
        else:
            disparity_cuda = algorithm.compute(left_cuda, right_cuda, cv2.cuda_Stream.Null())
            return disparity_cuda.download()