import numpy as np
from PIL import Image, ImageFilter
import matplotlib.pyplot as plt

# ------------------------- Perlin Noise Functions ------------------------- #
def generate_perlin_noise_2d(shape, res):
    """ http://staffwww.itn.liu.se/~stegu/simplexnoise/simplexnoise.pdf """
    def f(t):
        return 6*t**5 - 15*t**4 + 10*t**3

    delta = (res[0] / shape[0], res[1] / shape[1])
    d = (shape[0] // res[0], shape[1] // res[1])
    grid = np.mgrid[0:res[0]:delta[0],0:res[1]:delta[1]].transpose(1, 2, 0) % 1
    # Gradients
    angles = 2*np.pi*np.random.rand(res[0]+1, res[1]+1)
    gradients = np.dstack((np.cos(angles), np.sin(angles)))
    g00 = gradients[0:-1,0:-1].repeat(d[0], 0).repeat(d[1], 1)
    g10 = gradients[1:  ,0:-1].repeat(d[0], 0).repeat(d[1], 1)
    g01 = gradients[0:-1,1:  ].repeat(d[0], 0).repeat(d[1], 1)
    g11 = gradients[1:  ,1:  ].repeat(d[0], 0).repeat(d[1], 1)
    # Ramps
    n00 = np.sum(np.dstack((grid[:,:,0]  , grid[:,:,1]  )) * g00, 2)
    n10 = np.sum(np.dstack((grid[:,:,0]-1, grid[:,:,1]  )) * g10, 2)
    n01 = np.sum(np.dstack((grid[:,:,0]  , grid[:,:,1]-1)) * g01, 2)
    n11 = np.sum(np.dstack((grid[:,:,0]-1, grid[:,:,1]-1)) * g11, 2)
    # Interpolation
    t = f(grid)
    n0 = n00*(1-t[:,:,0]) + t[:,:,0]*n10
    n1 = n01*(1-t[:,:,0]) + t[:,:,0]*n11
    return np.sqrt(2)*((1-t[:,:,1])*n0 + t[:,:,1]*n1)

# -------------------------------------------------------------------------- #

def map(arr, from_range, to_range):
    """ Maps np.array from range to a certain range """

    return to_range[0] + (arr - from_range[0]) / float(np.diff(from_range)) * float(np.diff(to_range))

def generate_random_bitmap(shape=(256, 256), resolutions=[(8, 8)], mappings=[(-1,1)], img_size=(100, 100), file_name="height_map.bmp", save=True, return_noise=False):
    """ Generates a random bitmap (.bmp) file for use as a terrain

    Inputs:
        shape: Generated noise grid shape
            tuple(width, height)
        resolutions: list of resolutions to be used in perlin noise generation
            [tuple(res1, res1), tuple(res2, res2)...]
        mappings: list of mappings to be used to adjust the impact of certain noise generations
            [tuple(map1, map1), tuple(map2, map2)...]
        img_size: Image size
            tuple(width, height)
        file_name: File to save .bmp file to, only relevant if save=True
        save: Save the generated .bmp file
        return_grid: return the generated noise grid
    """

    if len(resolutions) != len(mappings):
        raise Exception('generate_random_bitmap :: Length of shapes list and resolutions list must be the same size! Exiting...')
    elif any(s % 2 for s in shape):
        raise Exception('generate_random_bitmap :: Shape must be even! Exiting...')

    length = len(resolutions)

    noise = None
    for res, mapping in zip(resolutions, mappings):
        # generate perlin noise with specified shape and resolution
        gen_noise = generate_perlin_noise_2d(shape=shape, res=res)

        # map values to a specific range
        mapped_noise = map(gen_noise, np.array([-1, 1]), np.array(mapping))

        # Add noises together
        if noise is None:
            noise = mapped_noise
        else:
            noise += mapped_noise

    # Map to image pixel range
    from_range = [np.min(noise), np.max(noise)]
    noise = map(noise, from_range, np.array([0,255]))

    # Create and save image
    img = Image.fromarray(np.uint8(noise))
    img = img.resize(img_size)
    img = img.filter(ImageFilter.GaussianBlur(8))
    if save:
        img.save(file_name)

    if return_noise:
        return noise

if __name__ == '__main__':
    show = False
    save = True

    np.random.seed(2)

    for _ in range(1):
        shape = (252, 252)
        noise = generate_random_bitmap(shape=shape, resolutions=[(2, 2)], mappings=[(-1.5,1.5)], img_size=(400,400), return_noise=True)
        if show:
            image = plt.imread("height_map.bmp")
            plt.imshow(image, cmap='gray', interpolation='lanczos')
            plt.show()
