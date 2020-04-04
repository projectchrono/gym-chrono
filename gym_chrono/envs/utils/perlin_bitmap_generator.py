import numpy as np
from PIL import Image

# ------------------------- Perlin Noise Functions ------------------------- #
def generate_perlin_noise_2d(shape, res):
    """ https://github.com/pvigier/perlin-numpy """
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

def generate_fractal_noise_2d(shape, res, octaves=1, persistence=0.5):
    """ https://github.com/pvigier/perlin-numpy """
    noise = np.zeros(shape)
    frequency = 1
    amplitude = 1
    for _ in range(octaves):
        noise += amplitude * generate_perlin_noise_2d(shape, (frequency*res[0], frequency*res[1]))
        frequency *= 2
        amplitude *= persistence
    return noise

# -------------------------------------------------------------------------- #

def map(arr, from_range, to_range):
    """ Maps np.array from range to a certain range """

    return to_range[0] + (arr - from_range[0]) / float(np.diff(from_range)) * float(np.diff(to_range))

def generate_random_bitmap():
    noise = generate_perlin_noise_2d(shape=(256, 256), res=(8, 8))
    noise = map(noise, np.array([-1,1]), np.array([0,255]))
    img = Image.fromarray(np.uint8(noise))
    width, height = img.size
    img = img.resize((int(width/2), int(height/2)))
    img.save("height_map.bmp")

if __name__ == '__main__':
    show = False

    np.random.seed(0)
    noise = generate_perlin_noise_2d((256, 256), (8, 8))
    if show:
        import matplotlib.pyplot as plt
        plt.imshow(noise, cmap='gray', interpolation='lanczos')
        plt.show()

    noise -= np.amin(noise)
    noise /= 2.0
    noise *= 255.0
    noise += 1

    img = Image.fromarray(np.uint8(noise))

    try:
         #Relative Path
        width, height = img.size
        print(width/2, height/2)
        img = img.resize((int(width/2), int(height/2)))

        #Saved in the same relative location
        img.save("resized_picture.bmp")
    except IOError:
        print('Fail')
