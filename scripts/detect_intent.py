#!/usr/bin/python

import sys
import numpy as np
import os.path
import scipy.spatial.distance as sd
from nltk.stem.lancaster import LancasterStemmer
import nltk

from config import params_setup

import rospkg
import rospy
from roboy_communication_cognition.srv import DetectIntent

rospack = rospkg.RosPack()
sys.path.insert(0, rospack.get_path('roboy_intents_classification'))

from include import skipthoughts

neighbors = 1
stemmer = LancasterStemmer()
words = []
classes = []
documents = []
sentences = []
ignore_words = ['?', ',', 'roboy', 'Roboy', '\n', '.']
args = params_setup()


def read_intents():
    import os
    intents_path = args.intents_path#os.getcwd() + "/intents/";
    training_data = []
    for filename in os.listdir(intents_path):
        with open(intents_path + filename) as f:
            for line in f:
                training_data.append({"class": filename, "sentence": line})

    return training_data


def sanitize_sentence(sentence):
    words = [stemmer.stem(w.lower()) for w in sentence if w not in ignore_words]
    sentence_new = "".join(str(x) for x in words)
    return sentence_new


def sanitize_dataset(training_data):
    # loop through each sentence in our training data
    for pattern in training_data:
        pattern['sentence'] = sanitize_sentence(pattern['sentence'])
        # add to our classes list
        if pattern['class'] not in classes:
            classes.append(pattern['class'])
    return classes


def get_nn(encoder, encodings, training_data, sentence):
    encoding = encoder.encode([sentence])
    encoding = encoding[0]
    scores = sd.cdist([encoding], encodings, "cosine")[0]
    sorted_ids = np.argsort(scores)
    print("Sentence : " + sentence)
    print("\nNearest neighbors:")
    for i in range(0, neighbors):
        print(" %d. %s (%.3f) %s" %
              (i + 1, sentences[sorted_ids[i]], scores[sorted_ids[i]], training_data[sorted_ids[i]]["class"]))
    return training_data[sorted_ids[i]]["class"], scores[sorted_ids[i]]


def init(sentence, args):
    training_data = read_intents()
    sanitize_dataset(training_data)
    for pattern in training_data:
        sentences.append(pattern['sentence'])
    model = skipthoughts.load_model(args)
    encoder = skipthoughts.Encoder(model)
    encodings = encoder.encode(sentences)
    sentence_sanitized = sanitize_sentence(sentence)
    intent = get_nn(encoder, encodings, training_data, sentence_sanitized)
    print(intent)

def get_intent(req):
    sentence_sanitized = sanitize_sentence(req.sentence)
    response = {}
    neighbor = get_nn(encoder, encodings, training_data, sentence_sanitized)
    response['intent'] = neighbor[0]
    response['distance'] = neighbor[1]
    return response

def main():

    global encoder, encodings, training_data, sentence_sanitized
    training_data = read_intents()
    sanitize_dataset(training_data)
    for pattern in training_data:
        sentences.append(pattern['sentence'])
    model = skipthoughts.load_model(args)
    encoder = skipthoughts.Encoder(model)
    encodings = encoder.encode(sentences)

    rospy.init_node('roboy_intents_classification')
    rospy.Service('/roboy/cognition/detect_intent', DetectIntent, get_intent)
    print "/roboy/cognition/detect_intent is ready"
    rospy.spin()


if __name__ == '__main__':
    main()
