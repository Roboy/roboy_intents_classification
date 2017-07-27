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
To download the datasets and install python requirements run:
```
./setup.sh
```
### ROS usage
Build the ROS package
```
source $ROS_WS/devel/setup.bash
catkin_make --pkg roboy_intents_classification
```
Start the node that offers intent classification service
```
roslaunch roboy_intents_classification intents.launch
```
Call the ROS service:
```
rosservice call /roboy/cognition/detect_intent "sentence: 'how old are you'"
```
Example output:
```
intent: age_intent
distance: 0.153419628739
```

### No ROS usage
Run basic command line demo:
```
cd scripts
python detect_intent.py --no-ros __name:='' __log:='' 
```

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
- [X] Add ROS endpoint.

## License

[Apache License 2.0](http://www.apache.org/licenses/LICENSE-2.0)
