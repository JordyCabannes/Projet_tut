import pymongo
from pymongo import MongoClient

client = MongoClient()
db=client.test_database
collection = db.test_collection


post = {"author": "Mike", "text": "My first blog post!"}
posts = db.posts
post_id = posts.insert(post)
post_id

#print db.collection_names()

a=posts.find_one({"author": "Mike"})

print a['author']