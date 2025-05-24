#!/bin/bash

#run with your message and version: For example

#./git_client_update.sh -m "Updated signal handling in FPLC_client.py" -v v0.1


# Function to display usage
usage() {
    echo "Usage: $0 -m <commit_message> -v <version_tag>"
    exit 1
}

# Parse command line arguments
while getopts ":m:v:" opt; do
    case $opt in
        m) commit_message="$OPTARG"
        ;;
        v) version_tag="$OPTARG"
        ;;
        *) usage
        ;;
    esac
done

# Check if both arguments are provided
if [ -z "$commit_message" ] || [ -z "$version_tag" ]; then
    usage
fi

# Navigate to the project directory
cd /home/sybednar/FPLC_client/FPLC_interface || { echo "Directory not found"; exit 1; }

# Stage the FPLC_client.py file
git add FPLC_client.py

# Commit the changes
git commit -m "$commit_message"

# Push the changes to GitHub
git push origin master

# Check if the tag already exists
if git rev-parse "$version_tag" >/dev/null 2>&1; then
    echo "Tag $version_tag already exists. Skipping tagging."
else
    # Tag the commit
    git tag "$version_tag" -m "$commit_message"
    git push origin "$version_tag"
fi

echo "Changes have been pushed and tagged with version $version_tag"
