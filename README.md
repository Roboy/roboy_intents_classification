# Sent2Vec (Skip-Thoughts)

Sent2Vec to get nearest neighbor from intent-utterances.
 
## Dependencies

This code is written in python. To use it you will need:

* Python 2.7
* Theano 0.7
* A recent version of [NumPy](http://www.numpy.org/) and [SciPy](http://www.scipy.org/)
* [scikit-learn](http://scikit-learn.org/stable/index.html)
* [NLTK 3](http://www.nltk.org/)
* [Keras](https://github.com/fchollet/keras) (for Semantic-Relatedness experiments only)
* [gensim](https://radimrehurek.com/gensim/) (for vocabulary expansion when training new models)

## Getting started

You will first need to download the model files and word embeddings to the model directory. The embedding files (utable and btable) are quite large (>2GB) so make sure there is enough space available. The encoder vocabulary can be found in dictionary.txt. 

    wget http://www.cs.toronto.edu/~rkiros/models/dictionary.txt
    wget http://www.cs.toronto.edu/~rkiros/models/utable.npy
    wget http://www.cs.toronto.edu/~rkiros/models/btable.npy
    wget http://www.cs.toronto.edu/~rkiros/models/uni_skip.npz
    wget http://www.cs.toronto.edu/~rkiros/models/uni_skip.npz.pkl
    wget http://www.cs.toronto.edu/~rkiros/models/bi_skip.npz
    wget http://www.cs.toronto.edu/~rkiros/models/bi_skip.npz.pkl


	
Run intent_classification_ros.py

![Alt text](nb.png?raw=true "IntentGrouping.ipynb")

Change the following as per application needs.
* data      --> List of all utterances (sentences of different intents) from which the nearest neighbor has to be computed.
* sentence  --> The utterance or sentence for which the most likely intent is to be found by looking up the nearest neighbors in the dataset.
* neighbors --> Number of nearest neighbors required to compute the semantic similarity of the given sentence. Default is set to 10

## Logic

The encoder first loads the downloaded pretrained models and word embeddings. Optionally, you can chose to train the model again on the application specific dictionary so as to tune the weights with the intents of the data under concern.

Then, it generates Skip-Thought Vectors for each sentence in the dataset.
	
**encodings = encoder.encode(data)**
	
encodings is a numpy array with as many rows as the length of X, and each row is 4800 dimensional (combine-skip model, from the paper). The first 2400 dimensions is the uni-skip model, and the last 2400 is the bi-skip model. It is highly recommend using the combine-skip vectors, as they are almost universally the best performing in the paper experiments.

The helper function **get_nn()** computes nearest neighbors of the given sentence in the dataset.

As the vectors are being computed, it will print some numbers. The code works by extracting vectors in batches of sentences that have the same length - so the number corresponds to the current length being processed. If you want to turn this off, set verbose=False when calling encode.


## Reference

This Sent2Vec encoder and training code is based on the paper [Skip-Thought Vectors](http://arxiv.org/abs/1506.06726).

Ryan Kiros, Yukun Zhu, Ruslan Salakhutdinov, Richard S. Zemel, Antonio Torralba, Raquel Urtasun, and Sanja Fidler. **"Skip-Thought Vectors."** *arXiv preprint arXiv:1506.06726 (2015).*

    @article{kiros2015skip,
      title={Skip-Thought Vectors},
      author={Kiros, Ryan and Zhu, Yukun and Salakhutdinov, Ruslan and Zemel, Richard S and Torralba, Antonio and Urtasun, Raquel and Fidler, Sanja},
      journal={arXiv preprint arXiv:1506.06726},
      year={2015}
    }

## To Do

- [X] Make changes to **sentence2vec.py** to pass dataset, sample sentence and no. of neighbors as arguments
- [X] Use the nearest neighbors to classify the most likely intent of the new sentence/utterance.
- [X] Add more neighbour sentences.
- [X] Add Class for every neighbour sentences.
- [ ] Evaluate.
- [ ] Add ROS endpoint.

## License

[Apache License 2.0](http://www.apache.org/licenses/LICENSE-2.0)
